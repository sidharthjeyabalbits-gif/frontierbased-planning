# frontier_explorer

Single-launch **frontier-based mapping of an unknown space** with TurtleBot3 in
Gazebo (ROS 2 Humble):

```
Gazebo + TB3  ──►  slam_toolbox  ──►  Nav2  ──►  explore_lite
 (sim + lidar)     (/map, map→odom)   (plan+ctrl)  (drive to frontiers)
```

`explore_lite` picks the nearest/largest boundary between free and unknown
space on the SLAM map, sends it to Nav2 as a goal, and repeats until no
frontiers remain — then drives back to the start pose.

## Build

```bash
cd ~/claude/fbm
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`explore_lite` is not packaged for Humble, so it is vendored as source at
`src/m-explore-ros2` (from https://github.com/robo-friends/m-explore-ros2)
and built by the same `colcon build`.

## Run

```bash
source ~/claude/fbm/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch frontier_explorer frontier_explore.launch.py
```

RViz comes up on the Nav2 default view. Startup sequence (all gated on real
readiness, see below): Gazebo + SLAM + Nav2 come up, the robot does one slow
360° turn to map its starting area, then `explore_lite` takes over and drives
frontier to frontier until the space is mapped, then returns to start.

### Options

| arg | default | meaning |
|---|---|---|
| `world_launch` | `turtlebot3_world.launch.py` | also `turtlebot3_house.launch.py`, `empty_world.launch.py` |
| `model` | `burger` | `burger` \| `waffle` \| `waffle_pi` |
| `rviz` | `true` | set `false` for headless |
| `nav2_delay` | `8.0` | seconds after slam_toolbox before Nav2 |
| `costmap_gate_delay` | `16.0` | seconds after slam_toolbox before the `wait_for_costmap` gate |
| `explore_autoresume` | `true` | run the `resume_explore` watchdog |
| `slam_params_file` / `nav2_params_file` / `explore_params_file` | package configs | override tuning |

```bash
ros2 launch frontier_explorer frontier_explore.launch.py \
    world_launch:=turtlebot3_house.launch.py model:=waffle_pi rviz:=false
```

### Save the finished map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/claude/fbm/maps/my_map
```

## Status by world

| world | result |
|---|---|
| `turtlebot3_world` (small arena) | **fully works** — explores and closes the map in ~50 s, saved map `../../maps/turtlebot3_world_fbm.*` |
| `turtlebot3_house` (multi-room) | **partial** — brings up cleanly, the robot autonomously spins then drives room-to-room mapping ~60 % of the house (~5500 cells) in 2–3 min, then it starts oscillating near a hard frontier (narrow doorway / furniture gap the inflated footprint won't cross) and the watchdog only creeps it forward. Saved partial map `../../maps/house_fbm_partial.*`. |

### Getting further in `turtlebot3_house`

The remaining problem is navigation, not the frontier logic: `explore_lite`
is a basic explorer and the stock `nav2_params.yaml` inflation makes the
burger too "fat" for some house doorways. Options, roughly in order of
effort:

* **Give it CPU headroom** — `gzserver` alone eats ~70 % of a core on the
  house world. Close other Gazebo/RViz/SLAM stacks and heavy apps; check
  `uptime` (load average should be **below your core count**); run
  `rviz:=false`; isolate with a distinct `ROS_DOMAIN_ID`. When the sim
  drops below ~1× real time, Nav2's BT action-ack timeouts trip and the
  robot stalls on top of the other issues.
* **Shrink the footprint / inflation** — a custom `nav2_params_file` with
  `robot_radius` ~0.12 and `inflation_layer` `inflation_radius` ~0.20 lets
  the burger through tight doorways.
* **Tune the explorer** — in `config/explore.yaml` lower `min_frontier_size`
  (0.75 → 0.4) so it keeps seeing small openings, raise `gain_scale` so it
  prefers big unexplored regions over nearby scraps, and raise
  `min_goal_distance` (0.5 → 0.8) so "push past the frontier" goals reach
  further into the unknown.
* **Swap `explore_lite`** for a wavefront frontier explorer
  (`nav2_wavefront_frontier_exploration`) or Nav2's own exploration
  behaviors — better goal selection and recovery in cluttered maps.

## How the ordering works (and why)

Layers are **event-driven, not fixed timers**. Four helpers make the chain
robust; none can be replaced by a plain `TimerAction`:

1. **`wait_for_sim.py`** — blocks until sim time is advancing **and** `/scan`
   is arriving **and** the `odom→base_footprint` TF resolves. On its exit,
   `slam_toolbox` starts. `async_slam_toolbox_node` started before Gazebo
   publishes `/clock` hangs silently forever (logs `Using solver plugin …`,
   never registers the laser).
2. Nav2 `navigation_launch.py` starts `nav2_delay` s later — planner +
   controller only; slam_toolbox already provides `map→odom`, so no AMCL /
   map_server.
3. **`wait_for_costmap.py`** — `costmap_gate_delay` s after slam, blocks
   until `/global_costmap/costmap` actually contains free space (i.e. Nav2 is
   genuinely up).
4. **`initial_spin.py`** — one slow 360° turn. A robot starting centered in a
   closed room sees a near-symmetric free↔unknown ring whose centroid is
   ≈ its own pose, so every `explore_lite` goal lands inside Nav2's
   `xy_goal_tolerance`, Nav2 reports instant success, the robot never moves,
   and `slam.yaml`'s `minimum_travel_distance` then freezes the map — a hard
   deadlock. The spin maps the whole starting room first and breaks the
   symmetry.
5. **`explore_lite`** starts when the spin finishes. It reads
   `costmap_topic: /map` (the raw slam_toolbox grid), **not** the Nav2 global
   costmap: this build of `explore_lite` is one-shot — the first empty
   frontier list calls `stop()` for good — and the global costmap briefly has
   no unknown-adjacent free cells during its rebuilds. `/map` grows
   monotonically and always has real borders until exploration is done.
6. **`resume_explore.py`** (watchdog, toggle `explore_autoresume`) — watches
   `/map`; if the known-cell count goes flat for two checks it publishes
   `explore/resume` (`std_msgs/Bool`) to re-arm `explore_lite`, up to 15
   times, then assumes the run is finished and exits. (It cannot rescue a
   robot that is stuck because *Nav2* is starved — see "bigger worlds".)

Manual resume, any time:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

## Files

| path | what |
|---|---|
| `launch/frontier_explore.launch.py` | the whole stack, event-chained |
| `scripts/wait_for_sim.py` | gate: sim clock + scan + TF ready |
| `scripts/wait_for_costmap.py` | gate: Nav2 global costmap has free space |
| `scripts/initial_spin.py` | one 360° turn to break the start-room symmetry |
| `scripts/resume_explore.py` | watchdog: re-arm one-shot explore_lite while the map grows |
| `config/slam.yaml` | slam_toolbox (mapping mode, LDS-01 3.5 m range) |
| `config/explore.yaml` | explore_lite (frontier scoring, `/map` source) |
