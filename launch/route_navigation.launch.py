#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动Nav2底层服务（map_server, amcl, planner_server, controller_server等）
不启动bt_navigator
route_navigator_node 注册 /navigate_to_pose，拦截RViz目标
route_navigator_node 内部直接调用 planner_server + controller_server

可选参数：
  map:=<地图文件>            默认使用 mycar_nav2 的 MYCAR.yaml
  params:=<Nav2参数文件>     默认使用 mycar_nav2 的 param_mini_tank.yaml
  route_file:=<路网GeoJSON>  默认使用内置 warehouse_routes.geojson
  launch_rviz:=true          启动RViz（默认false）
  use_sim_time:=false        使用仿真时间
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mycar_nav_dir = get_package_share_directory('mycar_nav2')
    route_planner_dir = get_package_share_directory('route_planner')
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')

    map_dir = os.path.join(mycar_nav_dir, 'map')
    param_dir = os.path.join(mycar_nav_dir, 'param', 'mycar_mppi_params')
    routes_dir = os.path.join(route_planner_dir, 'routes')
    config_dir = os.path.join(route_planner_dir, 'config')
    rviz_config = os.path.join(mycar_nav_dir, 'rviz', 'mycar.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_rviz = LaunchConfiguration('launch_rviz', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'MYCAR.yaml'))
    param_file = LaunchConfiguration('params', default=os.path.join(param_dir, 'param_mini_tank.yaml'))
    route_file = LaunchConfiguration('route_file', default=os.path.join(routes_dir, 'warehouse_routes.geojson'))
    config_file = os.path.join(config_dir, 'route_planner.yaml')

    declare_args = [
        DeclareLaunchArgument('map', default_value=map_file, description='地图文件路径'),
        DeclareLaunchArgument('params', default_value=param_file, description='Nav2参数文件路径'),
        DeclareLaunchArgument('route_file', default_value=route_file, description='GeoJSON路网文件路径'),
        DeclareLaunchArgument('launch_rviz', default_value='false', description='是否启动RViz2'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时钟'),
        DeclareLaunchArgument('autostart', default_value='true', description='是否自动启动Nav2生命周期'),
        DeclareLaunchArgument('allow_offroad', default_value='false',
            description='是否允许路网外自由导航（true=三段式，false=纯路网）'),
    ]

    # ---- 1. 机器人底层驱动 ----
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [wheeltec_launch_dir, '/turn_on_wheeltec_robot.launch.py']),
    )

    # ---- 2. 激光雷达 ----
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [wheeltec_launch_dir, '/wheeltec_lidar.launch.py']),
    )

    # ---- 2.5 摄像头 ----
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [wheeltec_launch_dir, '/wheeltec_camera.launch.py']),
    )

    # ================================================================
    #  3. Nav2 底层服务节点（不含bt_navigator）
    # ================================================================

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_file}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['map_server', 'amcl']}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            param_file,
            {'use_sim_time': use_sim_time},
            {
                'progress_checker': {
                    'plugin': 'nav2_controller::SimpleProgressChecker',
                    'required_movement_radius': 0.1,
                    'movement_time_allowance': 20.0,
                },
                'controller_plugins': ['FollowPath', 'FollowPathRPP'],
                'FollowPathRPP': {
                    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
                    'desired_linear_vel': 0.26,
                    'lookahead_dist': 0.4,
                    'min_lookahead_dist': 0.2,
                    'max_lookahead_dist': 0.6,
                    'lookahead_angle': 1.5708,
                    'use_velocity_scaled_lookahead_dist': False,
                    'min_approach_linear_velocity': 0.05,
                    'approach_velocity_scaling_dist': 0.6,
                    'use_collision_detection': True,
                    'max_allowed_time_to_collision_up_to_carrot': 1.0,
                    'max_allowed_time_to_collision': 2.0,
                    'max_angular_vel': 1.0,
                    'max_robot_pose_distance_to_carrot': 0.5,
                    'transform_tolerance': 0.2,
                    'xy_goal_tolerance': 0.15,
                    'yaw_goal_tolerance': 0.15,
                    'regulate_desired_linear_vel': True,
                    'regulate_desired_linear_vel_min': 0.05,
                    'regulate_desired_linear_vel_max': 0.26,
                },
            },
        ],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[param_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['controller_server', 'smoother_server',
                                    'planner_server', 'behavior_server',
                                    'velocity_smoother']}],
    )

    # ================================================================
    #  4. 路网导航节点
    # ================================================================
    route_navigator = Node(
        package='route_planner',
        executable='route_navigator_node',
        name='route_navigator_node',
        output='screen',
        parameters=[
            config_file,
            {
                'route_file': route_file,
                'map_frame': 'map',
                'robot_base_frame': 'base_link',
                'algorithm': 'astar',
                'publish_rate': 1.0,
                'marker_scale': 0.15,
                'snap_distance': 3.0,
                'on_graph_threshold': 0.5,
                'enabled': True,
                'use_sim_time': use_sim_time,
                'allow_offroad': LaunchConfiguration('allow_offroad'),
            }
        ],
    )

    # ---- 5. RViz（可选） ----
    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(declare_args + [
        # 机器人底层
        robot_launch,
        lidar_launch,
        camera_launch,
        # Nav2 底层服务
        map_server,
        amcl,
        lifecycle_manager_localization,
        planner_server,
        controller_server,
        behavior_server,
        smoother_server,
        velocity_smoother,
        lifecycle_manager_navigation,
        # 路网导航
        route_navigator,
        # 可视化
        rviz_node,
    ])
