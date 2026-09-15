# CA2 fix — testing guide

This covers how to verify the coordinate-conversion fix and maze-switching feature
described in `CLAUDE.md`, cross-checked against the official course deck
(`RB2301 - CA2 Opening.pptx`, not committed here — see `.gitignore`).

## What changed, in one paragraph

`path_planning_solution.py` (the reference solution) no longer hardcodes a separate
world-origin offset per environment. It derives `world_to_grid()`/`grid_to_world()` from
each maze's actual `origin`/`resolution` (`maze_config.py`), loaded from
`maze_0.yaml`/`maze_1.yaml`/`maze_sim.yaml`. A new `maze_id` ROS2 parameter selects
between the two real mazes at runtime. `path_planning.py` (the student stub students fill
in) is untouched — its `timer_callback()` is still empty, exactly as distributed.

## 0. One-time environment check

- `resolution: 0.2`, `origin: [-1.0, -5.0]` in `maze_sim.yaml`, and a 35×30 array in
  `ca2_sim_map.npy` — these match slide 26/27 exactly (`element[0,0]` = world `(-1.0,-5.0)`,
  `element[-1,-1]` = world `(5.8, 0.8)`). If you ever swap in a different sim world, these
  three numbers are what must change together.
- `maze_0.yaml`/`maze_1.yaml` point at the same `lab_maze_layout.yaml` (same map +
  goals-relative-to-origin) — only `origin` should ever differ between the two.

## 1. Build

```bash
cd ~/Documents/rb2301   # or wherever your workspace lives
colcon build --symlink-install
source install/setup.bash
```

If `colcon build` doesn't error, `maze_config.py`'s YAML/`.npy` files are correctly
packaged (verify with `find install/rb2301_ca2 -name "maze_*.yaml"` — should list 3 files
under `install/rb2301_ca2/lib/python3*/site-packages/rb2301_ca2/`, or similarly under
`lib/rb2301_ca2` if colcon used symlink-install's editable layout).

## 2. Reference-solution checks (no robot/sim needed)

These exercise the actual conversion math and maze-switching logic directly:

```bash
ros2 run rb2301_ca2 ca2_solution --ros-args -p maze_id:=0
```
Expect a startup log like:
```
Running on REAL ROBOT -- selected maze_id=0 (Physical lab maze on the OptiTrack floor (default maze)): 15x10 cells, resolution=0.2m, origin=(0.1, -1.9)
```
Ctrl-C to stop (it'll sit waiting for OptiTrack pose messages that aren't coming, which is
expected with nothing running yet).

```bash
ros2 run rb2301_ca2 ca2_solution --ros-args -p maze_id:=1
```
Same, but should log maze **1**'s description/origin.

```bash
ros2 run rb2301_ca2 ca2_solution --ros-args -p maze_id:=7
```
Should **fail immediately** with `Unknown maze_id 7. Available maze_ids: [0, 1]...` —
not silently fall back to maze 0.

```bash
ros2 run rb2301_ca2 ca2_solution
```
No `-p maze_id` at all — should default to maze 0.

## 3. Coordinate round-trip sanity check (standalone, no ROS needed)

```bash
cd src/rb2301_ca2/rb2301_ca2
python3 visualize_maze.py --maze 0 --point 2.1 -1.7
python3 visualize_maze.py --maze 1 --point 2.1 -1.7
python3 visualize_maze.py --maze sim --point 3.4 -3.6
```
Each should print `-> grid (ix, iy) -> value 0 (FREE)` and pop open a PNG with a green X
roughly in the middle of an open corridor, not on a wall. Try a point you know is a wall
(e.g. `--point 0 -1.9`, right on maze 0's boundary) and confirm it reports `WALL` or
`OUT OF BOUNDS`.

## 4. Simulation test (student workflow, per slide 38)

If you're testing over SSH/remote with no reachable display, use
`./gz_ca2.sh headless` instead of `./gz_ca2.sh` -- runs Gazebo server-only (no GUI
window), so it won't crash trying to connect to X. Physics and `/odom` work identically;
you just don't get the 3D viewer or `draw_grid_map()`'s popup images (those still try to
open a window and will error/no-op without a display -- safe to ignore, or check the
terminal logs / `ros2 topic echo /odom` instead).

```bash
./gz_ca2.sh        # terminal 1 -- launches Gazebo with the CA2 world (add "headless" if remote)
./ca2.sh           # terminal 2 -- runs YOUR path_planning.py code
# or, to test the reference solution instead of a student's own code:
./ca2_solution.sh  # terminal 2 -- runs path_planning_solution.py
```
- `is_simulation` must be `True` at the top of whichever file you're running (default).
- Reference solution should navigate through `sim_goal_list` in order —
  `(3.4,-3.6) → (3.2,0.2) → (2.4,-3.6) → (-0.4,-3.8)` — without clipping walls. Since
  `is_simulation=True`, `draw_grid_map()` fires each leg: a popup image with the planned
  path in green and waypoints in red should appear and roughly match the actual Gazebo
  layout you can see in the sim GUI.
- `maze_id` does **not** apply here (only one Gazebo world exists) — confirm the startup
  log says `maze_id parameter is not applicable in simulation`.

## 5. Real-robot test (per slides 39-42)

Before deploying:
1. In `path_planning_solution.py` (or your `path_planning.py`), set `is_simulation = False`.
2. Update the OptiTrack topic name to your assigned robot number:
   `/vrpn_mocap/bingda_00X/pose` (slide 40 — `X` is your robot's number, e.g. `bingda_003`).
3. Pick `maze_id` (0 or 1) matching whichever physical maze you're actually testing on.

```bash
ssh bingda@192.168.1.20X
cd ~/Downloads/rb2301_ca2_<YourName>
colcon build --symlink-install
source install/setup.bash
ros2 launch base_control_ros2 base_control.launch.py   # terminal A
vrpn                                                    # terminal B (OptiTrack bridge)
ros2 run rb2301_ca2 ca2_solution --ros-args -p maze_id:=0   # terminal C
```
Confirm the robot reaches all four real-world goals —
`(2.1,-1.7) → (2.3,-0.3) → (1.5,-1.7) → (0.3,-1.7)` — without touching the masking-tape
walls. If it clips a wall it previously would have reached fine (or vice versa), that's
the signal to re-check `maze_0.yaml`'s `origin` against where the maze is actually taped
down on the floor — see `README_ca2.md`'s "Viewing/editing a maze" section for how to
verify that with `visualize_maze.py` against a live OptiTrack point.

## 6. Full verification checklist (from CLAUDE.md)

- [x] Round-trip world→grid→world within half a cell, both maze configs — §3 above.
- [x] Border/margin cells excluded/included correctly, no off-by-one — checked via
      `visualize_maze.py`'s border render and automated bounds tests during development.
- [x] `maze_id:=0` and `maze_id:=1` load different configs and log clearly — §2 above.
- [x] No stale hardcoded-offset constants left anywhere else — grepped the whole `src/`
      tree; the only remaining `sim_grid_start`/`irl_grid_start`-style globals are in the
      untouched student stub `path_planning.py`, which never executed that math (its
      `timer_callback()` is empty) and isn't part of the live bug.

## Known limitations of this fix (be aware before grading/distributing)

- `maze_1.yaml`'s `origin` is currently a **placeholder**, identical to maze 0's — there's
  only one physical maze built right now. Update it (and nothing else) once the second
  maze's real OptiTrack origin is measured.
- I could not get a live, odometry-based confirmation that `ca2_sim_map.npy` matches the
  Gazebo world pixel-for-pixel (Gazebo Harmonic wasn't buildable in my sandboxed test
  environment — a Ubuntu/protobuf packaging gap, not a project bug). A static
  cross-check against the world file's wall positions was inconclusive and contradicted
  by the existing goal list, so I left the map data untouched. If you ever see the robot
  clip a sim wall it "shouldn't," that's the first thing to re-examine — see the chat
  history / `visualize_maze.py` for how to check it against a live Gazebo `/odom` reading.
