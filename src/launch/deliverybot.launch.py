#!/usr/bin/env python3
"""
ros2 launch parking_gz_sim deliverybot.launch.py

deliverybot ParkingLot 레이아웃 + CCTV 맵 파이프라인:
  Gazebo → CCTV 이미지 → cctv_map_builder(/parking/map)
        → map_astar_planner(A*, /goal_pose 수신) → sim_rigid_body_sync
목표 변경: RViz2의 'Goal Pose' 클릭 또는
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{pose: {position: {x: 25.0, y: 30.0}}}"
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('parking_gz_sim')
    world = os.path.join(pkg, 'worlds', 'deliverybot_parking.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge_deliverybot.yaml')
    sim_time = {'use_sim_time': True}

    # 이 월드의 로봇 배치 (SDF와 일치)
    robot_init = {
        'front_init_x': 3.25, 'front_init_y': -5.0,
        'rear_init_x': 0.75, 'rear_init_y': -5.0,
    }

    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': bridge_cfg}, sim_time],
             output='screen'),

        Node(package='parking_gz_sim', executable='cctv_map_builder',
             parameters=[sim_time, robot_init], output='screen'),

        Node(package='parking_gz_sim', executable='map_astar_planner',
             parameters=[sim_time, robot_init], output='screen'),

        Node(package='parking_gz_sim', executable='sim_rigid_body_sync',
             parameters=[sim_time, robot_init, {
                 'wheelbase': 2.5,        # 실물 차량 스케일
                 'lookahead': 1.5,
                 'max_speed': 1.0,
                 'max_omega': 0.5,
                 'goal_tolerance': 0.15,
                 'slow_radius': 2.0,
             }], output='screen'),
    ])
