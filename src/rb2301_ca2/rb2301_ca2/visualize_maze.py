#!/usr/bin/env python3
'''
Standalone dev tool to inspect and edit a maze's occupancy grid, and to check whether a
world-frame point (e.g. read off Gazebo or OptiTrack) lands on the cell you'd expect --
i.e. whether the .npy array still lines up with the real/simulated maze, or whether the
origin/resolution in maze_<id>.yaml has drifted from reality.

Needs no ROS2 / colcon build -- just numpy, PyYAML, matplotlib (run directly with
`python3 visualize_maze.py ...`).

Examples
--------
  # Render maze 0 to a PNG and open it in your image viewer
  python3 visualize_maze.py --maze 0

  # Check where world point (2.1, -1.7) lands -- prints the cell + occupancy value and
  # drops a marker on the render, so you can see at a glance whether it's where you expect
  python3 visualize_maze.py --maze 0 --point 2.1 -1.7

  # Same, but against the Gazebo sim map
  python3 visualize_maze.py --maze sim --point 3.4 -3.6

  # Flip cell (ix=5, iy=3) to a wall (99) and save -- backs up the .npy first
  python3 visualize_maze.py --maze 0 --set-cell 5 3 99
'''
import argparse
import shutil
import subprocess
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")  # render to a file; we open it with the OS viewer rather than a blocking window
import matplotlib.pyplot as plt

import maze_config as mc


def render(cfg, point=None, out_path=None):
    fig, ax = plt.subplots(figsize=(max(4, cfg.width / 4), max(4, cfg.height / 4)))

    # map_array is indexed [ix, iy] (axis0=x, axis1=y -- see maze_config.py). imshow wants
    # [row, col], so transpose (iy becomes row). With origin="lower" below, row 0 is drawn
    # at the BOTTOM of the axes (lowest y) and the last row at the top (highest y) -- which
    # is exactly what transposing alone gives (row r == iy == r), so no extra flip is
    # needed here. (An earlier version added np.flipud on top of this, which double-flipped
    # the image into a vertical mirror of the real array -- confirmed by rendering a
    # synthetic grid with one marked cell and sampling the actual pixel color under it.
    # The numeric --point / --set-cell paths read the array directly and were unaffected;
    # only the rendered picture was wrong.)
    display = cfg.map_array.T
    extent = [
        cfg.origin_x, cfg.origin_x + cfg.width * cfg.resolution,
        cfg.origin_y, cfg.origin_y + cfg.height * cfg.resolution,
    ]
    ax.imshow(display, cmap="gray_r", extent=extent, origin="lower", vmin=0, vmax=99)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    title = f"maze {cfg.label}: {cfg.description}"

    xticks = np.round(np.arange(extent[0], extent[1] + 1e-9, cfg.resolution), 6)
    yticks = np.round(np.arange(extent[2], extent[3] + 1e-9, cfg.resolution), 6)
    ax.set_xticks(xticks)
    ax.set_yticks(yticks)
    ax.tick_params(axis='x', labelrotation=90, labelsize=6)
    ax.tick_params(axis='y', labelsize=6)
    ax.grid(True, color="red", linewidth=0.3, alpha=0.4)

    if point is not None:
        px, py = point
        try:
            ix, iy = mc.world_to_grid(px, py, cfg)
            val = cfg.map_array[ix, iy]
            status = "WALL" if val != 0 else "FREE"
            ax.plot(px, py, marker="x", color="lime", markersize=16, markeredgewidth=3)
            title += f"\npoint ({px}, {py}) -> cell ({ix}, {iy}) = {val} [{status}]"
            print(f"world ({px}, {py}) -> grid ({ix}, {iy}) -> value {val} ({status})")
        except ValueError as e:
            ax.plot(px, py, marker="x", color="red", markersize=16, markeredgewidth=3)
            title += f"\npoint ({px}, {py}) -> OUT OF BOUNDS"
            print(f"OUT OF BOUNDS: {e}")

    ax.set_title(title, fontsize=9)
    out_path = out_path or f"/tmp/maze_{cfg.label}_render.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out_path


def load(maze_arg):
    if maze_arg == "sim":
        return mc.load_sim_config()
    return mc.load_maze_config(int(maze_arg))


def set_cell(cfg, ix, iy, value):
    if not (0 <= ix < cfg.width) or not (0 <= iy < cfg.height):
        sys.exit(f"cell ({ix},{iy}) is outside this maze's {cfg.width}x{cfg.height} grid")
    backup_path = f"{cfg.map_path}.bak.{int(time.time())}"
    shutil.copy2(cfg.map_path, backup_path)
    old_val = cfg.map_array[ix, iy]
    cfg.map_array[ix, iy] = value
    np.save(cfg.map_path, cfg.map_array)
    print(f"cell ({ix},{iy}): {old_val} -> {value}, saved to {cfg.map_path}")
    print(f"(backup of the original saved to {backup_path})")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--maze", required=True, help="maze_id (0, 1, ...) or 'sim'")
    parser.add_argument("--point", nargs=2, type=float, metavar=("X", "Y"), help="world-frame point to check")
    parser.add_argument("--set-cell", nargs=3, type=int, metavar=("IX", "IY", "VALUE"),
                         help="edit one grid cell and save (backs up the .npy first)")
    parser.add_argument("--out", help="output PNG path (default: /tmp/maze_<id>_render.png)")
    parser.add_argument("--no-open", action="store_true", help="don't try to open the rendered PNG in a viewer")
    args = parser.parse_args()

    cfg = load(args.maze)
    print(f"maze {cfg.label}: {cfg.description}")
    print(f"resolution={cfg.resolution}m origin=({cfg.origin_x},{cfg.origin_y}) size={cfg.width}x{cfg.height} cells")
    print(f"world extent: x[{cfg.origin_x}, {cfg.origin_x + cfg.width*cfg.resolution}] "
          f"y[{cfg.origin_y}, {cfg.origin_y + cfg.height*cfg.resolution}]")
    print(f"map file: {cfg.map_path}")
    print(f"config file: {cfg.config_path}")

    if args.set_cell:
        ix, iy, value = args.set_cell
        set_cell(cfg, ix, iy, value)
        cfg = load(args.maze)  # reload so the render below reflects the edit

    out_path = render(cfg, point=tuple(args.point) if args.point else None, out_path=args.out)
    print(f"rendered to {out_path}")

    if not args.no_open:
        try:
            subprocess.Popen(["xdg-open", out_path])
        except FileNotFoundError:
            print("(install xdg-open, or open the PNG yourself, to auto-preview)")


if __name__ == "__main__":
    main()
