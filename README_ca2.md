1. Extract compressed rb2301_ca2 folder to desired directory (e.g. ~/Documents)
2. Change directory to extracted folder then colcon build \
`cd ~/Documents/rb2301_ca2`\
`colcon build --symlink-install` \
3. Start the gazebo sim in a terminal while in the workspace folder \
`./gz_ca2.sh`\
If unable to run, you may need to assign permisions to run .sh files, if you have never done so. Do this with: \
`chmod +x *.sh`   
4. In another terminal, run the path planning script. \
`./ca2.sh` \
Add your code in the relevant portion within the rb2301_ca2 package's `path_planning.py`.

## Coordinate conversion (world <-> occupancy grid)

The planner works on a discrete occupancy grid, but the robot's pose (from Gazebo
odometry in sim, or OptiTrack on the real robot) is a continuous world-frame `(x, y)` in
meters. Converting between them needs two numbers for whichever maze is loaded:
`resolution` (meters per grid cell) and `origin` (the world-frame point where grid cell
`(0, 0)`'s corner sits).

- **World -> grid:** `ix = floor((x - origin_x) / resolution)`, same formula for `iy`.
- **Grid -> world:** `x = origin_x + (ix + 0.5) * resolution`, same formula for `y` --
  this returns the cell's *center*, not its corner, so a round trip lands within half a
  cell of the original point.

There is no separate hardcoded offset for sim vs. the real robot -- both use the same
`world_to_grid()`/`grid_to_world()` functions against whichever config is loaded (see
`rb2301_ca2/rb2301_ca2/maze_config.py`).

The two real mazes are physically identical structures placed at different spots on the
OptiTrack floor, so they share one occupancy grid + layout file
(`lab_maze_layout.yaml`: `resolution`, `map_file`, and each goal's position *relative to
that maze's own origin*), and each `maze_<id>.yaml` is a thin instance file naming only
`origin` (plus `maze_id`/`description`) -- the only thing that actually differs between
maze 0 and maze 1. The simulator's test room (`maze_sim.yaml`) is a physically different
room, so it stays self-contained.

## Selecting a maze (real robot only)

There are two real mazes tracked by the same shared OptiTrack world frame, selected at
runtime via the `maze_id` ROS2 parameter (default `0`):

```
ros2 run rb2301_ca2 path_planning --ros-args -p maze_id:=0   # your own code (path_planning.py)
ros2 run rb2301_ca2 path_planning --ros-args -p maze_id:=1

ros2 run rb2301_ca2 ca2_solution  --ros-args -p maze_id:=0   # reference solution
ros2 run rb2301_ca2 ca2_solution  --ros-args -p maze_id:=1
```

Or via the convenience scripts, passing the maze_id as the first argument (defaults to
`0` if omitted):

```
./ca2.sh 1            # your own code, maze 1
./ca2_solution.sh 1   # reference solution, maze 1
```

`maze_id` only applies to the real robot -- there is one Gazebo world for CA2, so
`./gz_ca2.sh` always uses the fixed simulation test room regardless of `maze_id`. An
unrecognised `maze_id` fails immediately at startup with a clear error rather than
silently defaulting.

To add a third physical copy of the same maze structure later, add a `maze_2.yaml`
(copy `maze_1.yaml` as a template) with its own `origin`, reusing `layout:
lab_maze_layout` -- no code changes, no new `.npy` needed. For a maze with a genuinely
different layout, also add a new layout YAML + `.npy` and point the new instance file's
`layout` at it.

## Viewing/editing a maze, and checking it matches reality

`rb2301_ca2/rb2301_ca2/visualize_maze.py` is a standalone script (no ROS2/colcon build
needed -- just `numpy`, `PyYAML`, `matplotlib`) for looking at what a maze's `.npy` array
actually contains, and for checking whether it still lines up with the real/simulated
maze after an edit:

```
cd src/rb2301_ca2/rb2301_ca2

# Render maze 0 (or 1, or sim) to a PNG and open it
python3 visualize_maze.py --maze 0

# Check where a world-frame point lands -- e.g. a spot you can see is a wall/free cell
# in Gazebo or on the physical maze -- and get a marker on the render at that point
python3 visualize_maze.py --maze sim --point 3.4 -3.6

# Edit one cell (e.g. mark it a wall) and save -- automatically backs up the .npy first
python3 visualize_maze.py --maze 0 --set-cell 5 3 99
```

To get a real-world point to check against: in simulation, run `./gz_ca2.sh`, look at
where a wall actually is in the Gazebo GUI, and cross-reference against
`ros2 topic echo /odom` for the robot's live ground-truth position; on the real robot,
use the OptiTrack pose (`ros2 topic echo /vrpn_mocap/bingda_003/pose`) the same way. If
`--point` reports FREE where you can see a wall (or vice versa), the map's
`origin`/`resolution` in the relevant `maze_<id>.yaml` no longer matches reality and
needs correcting there (not in code).
