"""
Maze configuration loading and world<->grid coordinate conversion for rb2301_ca2.

Coordinate frame convention
----------------------------
- World frame: continuous metric (x, y) in meters. On the real robot this is the
  OptiTrack world frame (shared by every maze); in simulation it's Gazebo's ground-truth
  odometry frame.
- Grid frame: discrete indices (ix, iy) into a maze's occupancy-grid numpy array.
  ``ix`` indexes axis 0 of the array, ``iy`` indexes axis 1. This is the OPPOSITE of the
  usual nav_msgs/OccupancyGrid row-major convention (row~y, col~x) -- it matches how the
  rest of this package's Grid/A*/draw_grid_map code already indexes the array as (x, y).
- Grid cell (0, 0)'s corner (its most-negative-x, most-negative-y corner) sits at world
  point (origin_x, origin_y). Cell (ix, iy) spans:
      world x in [origin_x + ix*resolution,     origin_x + (ix+1)*resolution)
      world y in [origin_y + iy*resolution,     origin_y + (iy+1)*resolution)
  "width"/"height" below mean the grid's extent in cells along the x-axis / y-axis
  respectively (i.e. width = map_array.shape[0], height = map_array.shape[1]) -- not the
  usual image width/height (which would be shape[1]/shape[0]).
- world_to_grid() floors to the containing cell (corner-referenced), matching how
  nav_msgs/OccupancyGrid.info.origin + resolution define cell (0,0).
- grid_to_world() returns the CENTER of a cell (a +0.5 cell offset), so a
  world -> grid -> world round trip lands within half a cell of the original point
  instead of snapping to a cell's corner.

Real mazes: one shared layout, per-instance origin
----------------------------------------------------
The two real mazes are physically identical structures, just placed at different spots
on the same OptiTrack floor -- so only their `origin` differs. To keep that a single
source of truth instead of two copies of the same map/goal data, each maze_<id>.yaml is
a thin "instance" file naming just `maze_id`, `description`, `origin`, and a `layout`
(e.g. ``lab_maze_layout.yaml``) that holds everything else once: `resolution`,
`map_file`, and `goal_list_local` (each goal's position *relative to that maze's own
origin*, i.e. as if origin were (0,0) -- this module adds the instance's actual origin
at load time to get world-frame goals). If a maze ever needs a genuinely different
layout, point its `layout` field at a new layout file instead of editing the shared one.

Adding a new maze
------------------
Drop a new ``maze_<id>.yaml`` file next to this module (copy ``maze_1.yaml`` as a
template) -- reusing the existing `layout` if it's physically the same structure, or
pointing at a new layout file (with its own `.npy`) otherwise. It is picked up
automatically the next time ``load_maze_config()``/``available_maze_ids()`` run -- no
code changes needed. The ``maze_id`` field inside the file must match the number in its
filename (checked at load time to catch copy-paste mistakes).

The simulation-only test room (``maze_sim.yaml``) is a physically different room (see
the Gazebo world file), so it's self-contained -- resolution/origin/map_file/goal_list
all in one file -- and loaded separately via ``load_sim_config()``. It's intentionally
not part of the maze_id-selectable set: there is currently only one Gazebo world, so
it isn't something a user picks at runtime.
"""
import glob
import os
import re

import numpy as np
import yaml
from dataclasses import dataclass


_CONFIG_DIR = os.path.dirname(os.path.realpath(__file__))
_MAZE_FILENAME_RE = re.compile(r"^maze_(\d+)\.yaml$")


@dataclass
class MazeConfig:
    '''Fully-resolved config for one maze: metadata plus the loaded occupancy grid.'''
    label: str            # e.g. "0", "1", or "sim" -- used only for logging/error messages
    description: str
    resolution: float     # meters per cell
    origin_x: float       # world-frame x of grid cell (0,0)'s corner, in meters
    origin_y: float       # world-frame y of grid cell (0,0)'s corner, in meters
    width: int            # grid extent along x-axis (= map_array.shape[0])
    height: int           # grid extent along y-axis (= map_array.shape[1])
    map_array: np.ndarray
    map_path: str         # absolute path to the .npy this map_array was loaded from
    config_path: str      # absolute path to the maze_<id>.yaml / maze_sim.yaml itself
    goal_list: list


def available_maze_ids() -> list:
    '''Return the sorted list of maze_ids that have a maze_<id>.yaml file next to this module.'''
    ids = []
    for path in glob.glob(os.path.join(_CONFIG_DIR, "maze_*.yaml")):
        match = _MAZE_FILENAME_RE.match(os.path.basename(path))
        if match:
            ids.append(int(match.group(1)))
    return sorted(ids)


def _read_yaml(path: str) -> dict:
    if not os.path.isfile(path):
        raise ValueError(f"Config file not found: {path}")
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _load_layout(layout_name: str, config_path: str) -> tuple:
    '''Load a shared layout file (resolution, occupancy grid, origin-relative goals).
    `config_path` is only used for error messages (which instance file referenced it).'''
    layout_path = os.path.join(_CONFIG_DIR, f"{layout_name}.yaml")
    raw = _read_yaml(layout_path)

    resolution = float(raw["resolution"])
    if resolution <= 0:
        raise ValueError(f"{layout_path}: resolution must be > 0, got {resolution}")

    map_file = os.path.join(_CONFIG_DIR, raw["map_file"])
    if not os.path.isfile(map_file):
        raise ValueError(f"{layout_path}: map_file {raw['map_file']!r} does not exist at {map_file}")
    map_array = np.load(map_file, allow_pickle=True)

    goal_list_local = [tuple(g) for g in raw.get("goal_list_local", [])]
    return resolution, map_array, map_file, goal_list_local, layout_path


def load_maze_config(maze_id: int) -> MazeConfig:
    '''Load and validate the maze config for the given maze_id: reads the thin
    maze_<id>.yaml instance file for {description, origin, layout}, then the referenced
    layout file for {resolution, map_file, goal_list_local} -- see the module docstring.

    Raises ValueError (with the list of valid maze_ids) if maze_id has no matching
    maze_<id>.yaml file -- callers should let this propagate rather than silently
    falling back to a default, so a bad maze_id fails loudly at startup.
    '''
    ids = available_maze_ids()
    if maze_id not in ids:
        raise ValueError(
            f"Unknown maze_id {maze_id!r}. Available maze_ids: {ids}. "
            f"To add a new maze, add a maze_{maze_id}.yaml file (see maze_1.yaml for the "
            f"required fields) in {_CONFIG_DIR}."
        )
    config_path = os.path.join(_CONFIG_DIR, f"maze_{maze_id}.yaml")
    raw = _read_yaml(config_path)

    if raw.get("maze_id") != maze_id:
        raise ValueError(
            f"{config_path} declares maze_id={raw.get('maze_id')!r}, but is named for "
            f"maze_id {maze_id}. Fix the mismatch before continuing."
        )

    origin_x, origin_y = raw["origin"]
    resolution, map_array, map_file, goal_list_local, layout_path = _load_layout(raw["layout"], config_path)
    width, height = map_array.shape
    goal_list = [(gx + origin_x, gy + origin_y) for gx, gy in goal_list_local]

    return MazeConfig(
        label=str(maze_id),
        description=raw.get("description", ""),
        resolution=resolution,
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        width=width,
        height=height,
        map_array=map_array,
        map_path=map_file,
        config_path=config_path,
        goal_list=goal_list,
    )


def load_sim_config() -> MazeConfig:
    '''Load the fixed simulation test-room config (maze_sim.yaml): self-contained (not
    split into instance+layout) since it's a physically different room from the real
    mazes, and not part of the maze_id-selectable set -- there is only one Gazebo world
    for CA2 right now.'''
    config_path = os.path.join(_CONFIG_DIR, "maze_sim.yaml")
    raw = _read_yaml(config_path)

    resolution = float(raw["resolution"])
    if resolution <= 0:
        raise ValueError(f"{config_path}: resolution must be > 0, got {resolution}")
    origin_x, origin_y = raw["origin"]

    map_file = os.path.join(_CONFIG_DIR, raw["map_file"])
    if not os.path.isfile(map_file):
        raise ValueError(f"{config_path}: map_file {raw['map_file']!r} does not exist at {map_file}")
    map_array = np.load(map_file, allow_pickle=True)
    width, height = map_array.shape
    goal_list = [tuple(g) for g in raw.get("goal_list", [])]

    return MazeConfig(
        label="sim",
        description=raw.get("description", ""),
        resolution=resolution,
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        width=width,
        height=height,
        map_array=map_array,
        map_path=map_file,
        config_path=config_path,
        goal_list=goal_list,
    )


_FLOOR_EPSILON = 1e-9  # guards against float64 subtraction noise landing just below a cell boundary


def world_to_grid(x: float, y: float, config: MazeConfig) -> tuple:
    '''Convert a world-frame point (meters) to grid indices (ix, iy), corner-referenced
    (floors to the containing cell). Raises ValueError if the point falls outside the grid.

    A small epsilon is added before flooring: e.g. with origin_y=-1.9, resolution=0.2,
    a point at y=-1.7 should floor to exactly cell index 1 ((-1.7 - -1.9) / 0.2 == 1.0),
    but float64 subtraction gives -1.7 - -1.9 == 0.19999999999999996, which floors to 0 --
    silently off by one cell. The epsilon is ~9 orders of magnitude smaller than any
    resolution used here, so it never misclassifies a point that's actually a fraction of
    a cell away from a boundary.
    '''
    ix = int(np.floor((x - config.origin_x) / config.resolution + _FLOOR_EPSILON))
    iy = int(np.floor((y - config.origin_y) / config.resolution + _FLOOR_EPSILON))
    if not (0 <= ix < config.width) or not (0 <= iy < config.height):
        raise ValueError(
            f"World point ({x}, {y}) maps to grid index ({ix}, {iy}), which is outside "
            f"maze {config.label}'s grid bounds (0 <= ix < {config.width}, 0 <= iy < {config.height})."
        )
    return ix, iy


def grid_to_world(ix: int, iy: int, config: MazeConfig) -> tuple:
    '''Convert grid indices (ix, iy) to the world-frame coordinate (meters) of that cell's
    CENTER (not corner). Raises ValueError if the index is outside the grid.'''
    if not (0 <= ix < config.width) or not (0 <= iy < config.height):
        raise ValueError(
            f"Grid index ({ix}, {iy}) is outside maze {config.label}'s grid bounds "
            f"(0 <= ix < {config.width}, 0 <= iy < {config.height})."
        )
    x = config.origin_x + (ix + 0.5) * config.resolution
    y = config.origin_y + (iy + 0.5) * config.resolution
    return x, y
