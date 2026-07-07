#!/usr/bin/env python3
"""
ros2 launch parking_gz_sim sim.launch.py

실행 순서:
  1. Gazebo Harmonic (parking_lot.sdf, 즉시 재생 -r)
  2. ros_gz_bridge (cmd_vel/odom/clock)
  3. waypoint_publisher (가짜 관제 — A* 경로 발행)
  4. sim_rigid_body_sync (강체 동기 제어)
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('parking_gz_sim')

    world = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'parking_lot.sdf'
    ])

    bridge_cfg = PathJoinSubstitution([
        pkg_share,
        'config',
        'bridge.yaml'
    ])

    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_cfg}, sim_time],
            output='screen',
        ),

        # CCTV image → OccupancyGrid /parking/map
        Node(
            package='parking_gz_sim',
            executable='cctv_map_builder',
            parameters=[sim_time],
            output='screen',
        ),

        # /parking/map → /virtual_robot/waypoints
        Node(
            package='parking_gz_sim',
            executable='map_astar_planner',
            parameters=[
                sim_time,
                {
                    'front_init_x': 0.75,
                    'front_init_y': 0.60,
                    'rear_init_x': 0.50,
                    'rear_init_y': 0.60,
                    'default_goal_x': 2.5,
                    'default_goal_y': 3.0,
                    'inflation': 0.15,
                    'plan_res': 0.2,
                }
            ],
            output='screen',
        ),

        # /virtual_robot/waypoints → front/rear cmd_vel
        Node(
            package='parking_gz_sim',
            executable='sim_rigid_body_sync',
            parameters=[sim_time],
            output='screen',
        ),
    ])