#!/usr/bin/env python3
"""
Single-file bring-up for frontier-based mapping of an unknown space
with TurtleBot3 in Gazebo.

Ordering is event-driven, not fixed timers:

    t=0        Gazebo + TurtleBot3        (turtlebot3_gazebo)
    t=0        RViz                        (nav2 default view)
    t=0        wait_for_sim                -> blocks until /clock + /scan + TF
    on ready   slam_toolbox (online async) -> /map, map->odom TF
    +nav2_delay        Nav2  (planner + controller, NO localization)
    +costmap_gate_delay wait_for_costmap   -> blocks until global costmap has free space
    on ready          initial_spin         -> one 360 deg turn to map the start room
    on done           explore_lite         -> drives to frontiers

Three helpers are load-bearing, do NOT swap any for a plain TimerAction:
  * slam_toolbox started before Gazebo publishes /clock hangs silently
    forever -> wait_for_sim.
  * explore_lite is one-shot on an empty frontier list; the Nav2 global
    costmap is briefly all-unknown right after activation -> wait_for_costmap.
  * a robot starting centered in a closed room sees a symmetric frontier
    ring whose centroid is ~its own pose, so every explore goal lands
    inside Nav2's goal tolerance and it never moves -> initial_spin breaks
    the symmetry before explore_lite starts.

Usage:
    export TURTLEBOT3_MODEL=burger
    ros2 launch frontier_explorer frontier_explore.launch.py
    ros2 launch frontier_explorer frontier_explore.launch.py \
        world_launch:=turtlebot3_house.launch.py rviz:=false model:=waffle_pi
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('frontier_explorer')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # ---- launch arguments -----------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_launch = LaunchConfiguration('world_launch')
    tb3_model = LaunchConfiguration('model')        # burger | waffle | waffle_pi
    use_rviz = LaunchConfiguration('rviz')
    slam_params = LaunchConfiguration('slam_params_file')
    explore_params = LaunchConfiguration('explore_params_file')
    nav2_params = LaunchConfiguration('nav2_params_file')
    nav2_delay = LaunchConfiguration('nav2_delay')
    costmap_gate_delay = LaunchConfiguration('costmap_gate_delay')
    autoresume = LaunchConfiguration('explore_autoresume')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world_launch', default_value='turtlebot3_world.launch.py',
            choices=['turtlebot3_world.launch.py',
                     'turtlebot3_house.launch.py',
                     'empty_world.launch.py'],
            description='launch file inside turtlebot3_gazebo/launch to run'),
        DeclareLaunchArgument('model', default_value='burger',
                              choices=['burger', 'waffle', 'waffle_pi']),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam.yaml')),
        DeclareLaunchArgument(
            'explore_params_file',
            default_value=os.path.join(pkg_share, 'config', 'explore.yaml')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav2_bringup, 'params', 'nav2_params.yaml'),
            description='Default nav2_bringup params are TurtleBot3-tuned'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='8.0',
            description='seconds after slam_toolbox starts before Nav2 starts'),
        DeclareLaunchArgument(
            'costmap_gate_delay', default_value='16.0',
            description='seconds after slam_toolbox starts before wait_for_costmap '
                        '(the explore_lite gate) starts; must be > nav2_delay so Nav2 '
                        'is activating by then'),
        DeclareLaunchArgument(
            'explore_autoresume', default_value='true',
            description='run resume_explore.py, which re-arms explore_lite if it '
                        'stops (one-shot) while the map is still growing'),
    ]

    # ---- make sure TB3 model / gazebo model path are set ---------------
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model)
    set_gz_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(tb3_gazebo, 'models') + os.pathsep
        + os.environ.get('GAZEBO_MODEL_PATH', ''))

    # ---- t=0 : Gazebo + TurtleBot3 -----------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(tb3_gazebo, 'launch'), '/', world_launch]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- t=0 : RViz --------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # ---- t=0 : wait for the simulation to actually be up -------------
    wait_for_sim = Node(
        package='frontier_explorer',
        executable='wait_for_sim.py',
        name='wait_for_sim',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ---- on ready : slam_toolbox (online async) ---------------------
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ---- +nav2_delay : Nav2 (planner + controller, NO localization) --
    # slam_toolbox already publishes map->odom, so we run navigation_launch.py
    # (not bringup_launch.py, which would also start amcl / map_server).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
    )

    # ---- +costmap_gate_delay : wait until global costmap has free space
    wait_for_costmap = Node(
        package='frontier_explorer',
        executable='wait_for_costmap.py',
        name='wait_for_costmap',
        parameters=[{
            'use_sim_time': use_sim_time,
            'costmap_topic': '/global_costmap/costmap',
        }],
        output='screen',
    )

    # ---- on costmap ready : one 360 deg turn to map the start room --
    initial_spin = Node(
        package='frontier_explorer',
        executable='initial_spin.py',
        name='initial_spin',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ---- after the spin : explore_lite -------------------------
    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[explore_params, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # watchdog: re-arm explore_lite if it stops while /map is still growing
    resume_explore = Node(
        package='frontier_explorer',
        executable='resume_explore.py',
        name='resume_explore',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(autoresume),
        output='screen',
    )

    start_slam_after_sim_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_sim,
            on_exit=[
                LogInfo(msg='[frontier_explorer] sim ready -> starting slam_toolbox'),
                slam,
            ],
        )
    )

    start_nav2_and_gate_after_slam = RegisterEventHandler(
        OnProcessStart(
            target_action=slam,
            on_start=[
                TimerAction(period=nav2_delay, actions=[
                    LogInfo(msg='[frontier_explorer] starting Nav2'),
                    nav2,
                ]),
                TimerAction(period=costmap_gate_delay, actions=[
                    LogInfo(msg='[frontier_explorer] waiting for global costmap'),
                    wait_for_costmap,
                ]),
            ],
        )
    )

    spin_after_costmap = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_costmap,
            on_exit=[
                LogInfo(msg='[frontier_explorer] costmap ready -> initial spin'),
                initial_spin,
            ],
        )
    )

    start_explore_after_spin = RegisterEventHandler(
        OnProcessExit(
            target_action=initial_spin,
            on_exit=[
                LogInfo(msg='[frontier_explorer] spin done -> starting explore_lite'),
                explore,
                resume_explore,
            ],
        )
    )

    return LaunchDescription([
        *declare_args,
        set_model,
        set_gz_path,
        gazebo,
        rviz,
        wait_for_sim,
        start_slam_after_sim_ready,
        start_nav2_and_gate_after_slam,
        spin_after_costmap,
        start_explore_after_spin,
    ])
