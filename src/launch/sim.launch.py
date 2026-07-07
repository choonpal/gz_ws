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
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('parking_gz_sim')
    world = os.path.join(pkg, 'worlds', 'parking_lot.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
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

        Node(
            package='parking_gz_sim',
            executable='waypoint_publisher',
            parameters=[sim_time],
            output='screen',
        ),

        Node(
            package='parking_gz_sim',
            executable='sim_rigid_body_sync',
            parameters=[sim_time],
            output='screen',
        ),
    ])
