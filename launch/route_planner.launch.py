#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路网规划导航节点启动文件

仅启动 route_planner 节点本身；其依赖的 Nav2 服务
（map_server / amcl / planner_server / controller_server）需另行启动，
可用 route_navigation.launch.py 拉起完整导航栈。

使用方法：
  ros2 launch route_planner route_planner.launch.py
  ros2 launch route_planner route_planner.launch.py route_file:=/path/to/custom.geojson
  ros2 launch route_planner route_planner.launch.py algorithm:=dijkstra
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('route_planner')
    config_dir = os.path.join(pkg_dir, 'config')
    routes_dir = os.path.join(pkg_dir, 'routes')

    default_config = os.path.join(config_dir, 'route_planner.yaml')
    default_route_file = os.path.join(routes_dir, 'warehouse_routes.geojson')

    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=default_route_file,
        description='GeoJSON路网文件路径'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='参数配置文件路径'
    )

    algorithm_arg = DeclareLaunchArgument(
        'algorithm',
        default_value='astar',
        description='路径规划算法: astar 或 dijkstra'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='地图坐标系名称'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='可视化发布频率(Hz)'
    )

    marker_scale_arg = DeclareLaunchArgument(
        'marker_scale',
        default_value='0.15',
        description='标记大小缩放系数'
    )

    route_file = LaunchConfiguration('route_file')
    config_file = LaunchConfiguration('config_file')
    algorithm = LaunchConfiguration('algorithm')
    map_frame = LaunchConfiguration('map_frame')
    publish_rate = LaunchConfiguration('publish_rate')
    marker_scale = LaunchConfiguration('marker_scale')

    planner_node = Node(
        package='route_planner',
        executable='route_navigator_node',
        name='route_planner_node',
        output='screen',
        parameters=[
            config_file,
            {
                'route_file': route_file,
                'map_frame': map_frame,
                'algorithm': algorithm,
                'publish_rate': publish_rate,
                'marker_scale': marker_scale,
            }
        ],
        remappings=[
            ('route_path', '/route_path'),
            ('route_graph_markers', '/route_graph_markers'),
        ]
    )

    return LaunchDescription([
        route_file_arg,
        config_file_arg,
        algorithm_arg,
        map_frame_arg,
        publish_rate_arg,
        marker_scale_arg,
        planner_node,
    ])
