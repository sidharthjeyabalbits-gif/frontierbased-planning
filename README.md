# frontierbased-planning

Autonomous **frontier-based exploration and mapping** of an unknown space with
TurtleBot3 in Gazebo — ROS 2 Humble.

```
Gazebo + TB3  ──►  slam_toolbox  ──►  Nav2  ──►  explore_lite
 (sim + lidar)     (/map, map→odom)   (plan+ctrl)  (drive to frontiers)
```

One launch file brings up the simulator, online SLAM, navigation, an initial
observation spin, and the frontier explorer, in dependency order. The robot
drives itself frontier to frontier until the reachable space is mapped, then
returns to its start pose.

## Layout

| path | what |
|---|---|
| `src/frontier_explorer/` | the integration package — launch file, configs, startup-gate helper nodes. **See its [README](src/frontier_explorer/README.md) for full docs.** |
| `src/m-explore-ros2/` | `explore_lite` vendored from [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) (no Humble release), with a local patch to `explore.cpp` so it pushes goals past a frontier the robot is already sitting on |
| `maps/` | example output grids |

## Quick start

```bash
git clone https://github.com/sidharthjeyabalbits-gif/frontierbased-planning.git ~/fbm_ws
cd ~/fbm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch frontier_explorer frontier_explore.launch.py
# bigger world:
ros2 launch frontier_explorer frontier_explore.launch.py \
    world_launch:=turtlebot3_house.launch.py
```

## Status

* `turtlebot3_world` — **fully autonomous**, closes the map in ~50 s
* `turtlebot3_house` — explores ~60 % autonomously, then needs Nav2
  footprint/inflation tuning to clear the tighter doorways
  (see `src/frontier_explorer/README.md`)
