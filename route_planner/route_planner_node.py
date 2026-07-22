#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路网导航规划节点

核心设计：三段式分段导航
  1. 上路（自由导航）-> planner_server 规划路径 + controller_server 跟踪
  2. 路网（逐节点跟随）-> 直接构建路径 + controller_server.FollowPath（跳过 planner）
  3. 下路（自由导航）-> planner_server 规划路径 + controller_server 跟踪

架构：
  本节点注册 /navigate_to_pose action server，内部直接调用
  planner_server.ComputePathToPose 与 controller_server.FollowPath，
  不依赖 bt_navigator，规避其命名空间冲突。
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import os
import threading

import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from action_msgs.msg import GoalStatus

from .geojson_parser import GeoJSONRouteParser
from .route_graph import RouteGraph
from nav2_msgs.msg import Costmap


class RouteNavigatorNode(Node):
    def __init__(self):
        super().__init__('route_navigator_node')

        # ---- 参数 ----
        self.declare_parameter('route_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('marker_scale', 0.15)
        self.declare_parameter('snap_distance', 3.0)
        self.declare_parameter('on_graph_threshold', 0.5)
        self.declare_parameter('enabled', True)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('dense_step', 0.2)
        self.declare_parameter('planner_timeout', 10.0)
        self.declare_parameter('controller_timeout', 60.0)
        self.declare_parameter('network_controller_id', 'FollowPathRPP')
        self.declare_parameter('allow_offroad', False)
        # ---- 障碍物动态重规划参数 ----
        self.declare_parameter('replan_enabled', True)
        self.declare_parameter('cost_threshold', 60)
        # 触发阈值：前方 cost_lookahead 内累计高代价点数达到该值即判堵塞。
        self.declare_parameter('path_obstacles_threshold', 2)
        self.declare_parameter('monitor_hz', 2.0)
        # 仅检查机器人前方该距离(米)内的路径点
        self.declare_parameter('cost_lookahead', 1.5)
        # Nav2 Humble: /global_costmap/costmap 是 nav_msgs/OccupancyGrid(0~100)，
        # 内部 0~255 的 nav2_msgs/Costmap 在 /global_costmap/costmap_raw。
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('replan_reverse_via_freenav', True)
        self.declare_parameter('replan_on_failure', True)
        # 单次导航最多重规划次数，防止极端情况下无限重规划
        self.declare_parameter('replan_max_attempts', 5)
        # 重规划触发模式：'both'=代价地图监控+控制器中止兜底都触发（默认）；
        # 'monitor'=仅代价地图监控触发；'failure'=仅控制器中止兜底触发。
        self.declare_parameter('replan_trigger_mode', 'both')

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.allow_offroad = self.get_parameter('allow_offroad').value
        self.algorithm = self.get_parameter('algorithm').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.snap_distance = self.get_parameter('snap_distance').value
        self.on_graph_threshold = self.get_parameter('on_graph_threshold').value
        self.enabled = self.get_parameter('enabled').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.dense_step = self.get_parameter('dense_step').value
        self.planner_timeout = self.get_parameter('planner_timeout').value
        self.controller_timeout = self.get_parameter('controller_timeout').value
        self.network_controller_id = self.get_parameter('network_controller_id').value
        self.replan_enabled = self.get_parameter('replan_enabled').value
        self.cost_threshold = self.get_parameter('cost_threshold').value
        self.path_obstacles_threshold = self.get_parameter('path_obstacles_threshold').value
        self.monitor_hz = self.get_parameter('monitor_hz').value
        self.cost_lookahead = self.get_parameter('cost_lookahead').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.replan_reverse_via_freenav = self.get_parameter('replan_reverse_via_freenav').value
        self.replan_on_failure = self.get_parameter('replan_on_failure').value
        self.replan_max_attempts = self.get_parameter('replan_max_attempts').value
        # 由触发模式推导两组布尔开关（replan_enabled 为总开关；
        # replan_on_failure 作为 failure 通道的向后兼容开关，设 false 可单独关掉兜底）。
        self.replan_trigger_mode = self.get_parameter('replan_trigger_mode').value
        _mode = self.replan_trigger_mode
        self._trigger_monitor = bool(self.replan_enabled and _mode in ('monitor', 'both'))
        self._trigger_failure = bool(self.replan_enabled and _mode in ('failure', 'both')
                                     and self.replan_on_failure)

        # ---- TF2 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 路网数据 ----
        self.parser = GeoJSONRouteParser()
        self.graph = RouteGraph()
        self.loaded = False
        self.current_path_nodes = []
        self.current_dense_poses = []

        # ---- Action Clients ----
        self.callback_group = ReentrantCallbackGroup()

        # 对外注册 /navigate_to_pose（接收RViz Nav2 Goal）
        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute_navigate,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group
        )

        # 内部连接 planner_server（全局路径规划）
        self._compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=self.callback_group
        )

        # 内部连接 controller_server（路径跟踪）
        self._follow_path_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=self.callback_group
        )

        # 内部“回环”客户端：把话题目标转发给本节点的 /navigate_to_pose，复用同一套导航逻辑
        self._nav_ac = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        self._active_nav = None              # 当前正在执行的 nav goal（同一时刻仅一个）
        self._pending_nav = None             # 抢占后等待串行启动的新 nav goal
        self._active_compute_goal = None    # 正在执行的 ComputePathToPose 内层 goal
        self._active_follow_goal = None     # 正在执行的 FollowPath(controller) 内层 goal
        self._goal_lock = threading.Lock()

        # ---- 发布者 ----
        self.marker_pub = self.create_publisher(MarkerArray, 'route_graph_markers', 10)
        self.route_path_pub = self.create_publisher(NavPath, 'route_path', 10)

        # ---- 话题入口（RViz “2D Goal Pose” 工具下发目标）----
        # 同时订阅两个话题，无论用默认 /goal_pose 还是 /route_planner/goal 都能收到。
        self._goal_pose_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self._on_goal_pose, 10,
            callback_group=self.callback_group)
        self._route_goal_sub = self.create_subscription(
            PoseStamped, 'route_planner/goal', self._on_goal_pose, 10,
            callback_group=self.callback_group)
        self._gp_result_future = None

        # ---- 定时器 ----
        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)

        # ---- 障碍物动态重规划相关状态 ----
        self._replan_requested = False        # 监控器置位，请求重规划
        self._replan_attempts = 0             # 本次导航已重规划次数(防无限循环)
        self._nav_active = False              # 是否正在执行分段导航
        self._current_task_type = ''          # 当前执行段类型('free'/'network')
        self._goal_node_id = None             # 本次导航的目标路网节点
        self._goal_yaw = 0.0                  # 用户设定的目标朝向
        self._blocked_seg_start = None        # 堵塞边起点节点
        self._blocked_seg_next = None         # 堵塞边终点节点
        self._dense_node_ids = []             # current_dense_poses 各点所属节点 id
        self._costmap_data = None             # 缓存的全局代价地图数据(int8)
        self._costmap_meta = None             # 代价地图元数据(resolution/origin/size)
        self._costmap_warn_done = False       # 代价地图未收到的告警标记
        self._costmap_warn_ts = 0.0           # 上次告警时间戳(每10秒重复提醒)
        self._costmap_received = False        # 是否已收到过代价地图

        # 订阅全局代价地图（仅 monitor 路径需要；failure 模式不订阅）
        self._costmap_sub = None
        if self._trigger_monitor:
            self._costmap_sub = self.create_subscription(
                Costmap, self.costmap_topic, self._costmap_callback, 10,
                callback_group=self.callback_group)
            self.get_logger().info(
                f'已启用代价地图监控路径，订阅 {self.costmap_topic}（nav2_msgs/Costmap）')
        else:
            self.get_logger().info(
                f'重规划触发模式为 {self.replan_trigger_mode!r}，不订阅代价地图'
                f'（仅控制器中止兜底路径生效）')

        # 障碍监控定时器（仅在路网段激活时检测）
        monitor_period = 1.0 / max(self.monitor_hz, 0.5)
        self._monitor_timer = self.create_timer(
            monitor_period, self._monitor_callback, callback_group=self.callback_group)

        self._load_route_graph()

        self.get_logger().info('路网导航节点已启动 [ActionServer: /navigate_to_pose]')
        self.get_logger().info('直接调用: /planner_server/compute_path_to_pose')
        self.get_logger().info('         + /controller_server/follow_path')
        self.get_logger().info('目标下发请用 RViz “2D Goal Pose” 工具；不要用 “Navigation2 Goal” 或 “2D Pose Estimate” 设目标')
        self.get_logger().info(f'机器人坐标系: {self.robot_base_frame}')
        if self.loaded:
            self.get_logger().info(f'路网已加载: {len(self.graph.nodes)}节点/{len(self.graph.edges)}边')
        else:
            self.get_logger().warn('路网未加载，退化为标准Nav2自由导航')

    # ========== /goal_pose 话题入口 ==========

    def _on_goal_pose(self, msg):
        """话题目标(/goal_pose 或 /route_planner/goal)入口，由 RViz
        “2D Goal Pose”(SetGoal) 工具触发，回环转发给本节点 /navigate_to_pose。
        """
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.get_logger().info(
            f'收到话题导航目标: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}) -> 转发至 /navigate_to_pose')
        send_future = self._nav_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_pose_sent)

    def _on_goal_pose_sent(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'/goal_pose 转发目标失败: {e}')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('/goal_pose 目标被服务器拒绝（可能已有导航在抢占中）')
            return
        # 内部客户端无需处理反馈/结果，但必须持有句柄引用直到结束，
        # 否则 rclpy 可能在 GC 时取消该目标。
        self._gp_result_future = goal_handle.get_result_async()
        self._gp_result_future.add_done_callback(
            lambda f: self._on_goal_pose_done(goal_handle))

    def _on_goal_pose_done(self, goal_handle):
        status = goal_handle.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('/goal_pose 导航成功完成')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('/goal_pose 导航被取消/抢占')
        else:
            self.get_logger().warn(f'/goal_pose 导航结束，状态: {status}')

    # ========== Action 回调 ==========

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        """新目标到达：串行化抢占，同一时刻仅一个 _execute_navigate 协程。

        有活动导航则将其 abort 并暂存新目标到 _pending_nav；旧协程 finally 收尾时
        串行驶启动 _pending_nav。无活动导航则直接 execute 新目标。
        """
        with self._goal_lock:
            try:
                if self._active_nav is not None and self._active_nav is not goal_handle:
                    old = self._active_nav
                    if old.is_active:
                        self._pending_nav = goal_handle
                        self.get_logger().info('新导航目标到达，抢占当前导航任务')
                        old.abort()
                        # 取消旧目标正在跑的内层 goal，立即停车并解除 await 阻塞
                        if self._active_follow_goal is not None:
                            self._active_follow_goal.cancel_goal_async()
                        if self._active_compute_goal is not None:
                            self._active_compute_goal.cancel_goal_async()
                    # old 已终止：其 finally 收尾会启动既有 _pending_nav，此处不覆盖
                elif self._active_nav is None:
                    # 当前无活动导航，直接启动
                    self._active_nav = goal_handle
                    if goal_handle.is_active:
                        goal_handle.execute()
                # 同一 goal 重复 accepted 的情况忽略
            except Exception as e:
                self.get_logger().error(f'处理新目标异常: {e}')

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        if self._active_follow_goal is not None:
            self._active_follow_goal.cancel_goal_async()
        if self._active_compute_goal is not None:
            self._active_compute_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _is_preempted(self, goal_handle):
        """该目标是否应停止执行：已失活（被 abort）或被请求取消。

        串行化后，抢占表现为 old.abort() 使 is_active 变 False，
        故只需检查这两个状态即可，无需再比较“当前 nav goal”。
        """
        if not goal_handle.is_active:
            return True
        if goal_handle.is_cancel_requested:
            return True
        return False

    def _terminate_goal(self, goal_handle, success):
        """安全终止目标：按当前状态选择 canceled/succeed/abort，避免无效状态转换"""
        if not goal_handle.is_active:
            return
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

    async def _execute_navigate(self, goal_handle):
        goal_pose = goal_handle.request.pose
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        goal_yaw = self._pose_to_yaw(goal_pose)
        self._goal_yaw = goal_yaw

        self.get_logger().info(f'收到导航目标: ({goal_x:.2f}, {goal_y:.2f})')
        self._send_feedback(goal_handle, '规划中...', 0)
        try:

            # 透传模式
            if not self.loaded or not self.enabled:
                self.get_logger().info('路网导航未启用，退化为自由导航')
                result = await self._free_navigate(goal_pose)
                self._terminate_goal(goal_handle, result)
                return NavigateToPose.Result()

            # ---- 获取机器人当前位姿 ----
            robot_pose = self._get_robot_pose()
            if robot_pose is None:
                self.get_logger().warn('无法获取机器人位姿，退化为自由导航')
                result = await self._free_navigate(goal_pose)
                self._terminate_goal(goal_handle, result)
                return NavigateToPose.Result()

            robot_x = robot_pose.pose.position.x
            robot_y = robot_pose.pose.position.y
            self.get_logger().info(f'机器人当前位置: ({robot_x:.2f}, {robot_y:.2f})')

            robot_on_graph = self._is_pose_on_graph(robot_x, robot_y)
            goal_on_graph = self._is_pose_on_graph(goal_x, goal_y)
            self.get_logger().info(f'机器人在路网: {robot_on_graph}, 目标在路网: {goal_on_graph}')

            start_node = self.graph.find_nearest_node(robot_x, robot_y, self.snap_distance)
            goal_node = self.graph.find_nearest_node(goal_x, goal_y, self.snap_distance)
            self.get_logger().info(f'最近入口节点: {start_node}, 最近出口节点: {goal_node}')

            if goal_node is None and not goal_on_graph:
                self.get_logger().warn(f'终点离路网太远(>{self.snap_distance}m)，直接自由导航')
                result = await self._free_navigate(goal_pose)
                self._terminate_goal(goal_handle, result)
                return NavigateToPose.Result()

            if goal_node is None and goal_on_graph:
                goal_node = self.graph.find_nearest_node(goal_x, goal_y, float('inf'))

            if goal_node is None:
                self.get_logger().warn('无法确定目标附近的路网节点，自由导航')
                result = await self._free_navigate(goal_pose)
                self._terminate_goal(goal_handle, result)
                return NavigateToPose.Result()

            # ---- 构建导航任务列表 ----
            # 根据 allow_offroad 决定：纯路网 或 三段式（上路+路网+下路）
            nav_tasks = []  # [(phase_name, task_type, data)]
            #   task_type: 'free' = planner规划+controller跟踪
            #              'network' = 直接构建路径+controller跟踪

            if not self.allow_offroad:
                # ========== 纯路网模式 ==========
                if not robot_on_graph:
                    self.get_logger().error('机器人不在路网上，纯路网模式拒绝导航')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                if not goal_on_graph:
                    self.get_logger().error('目标不在路网上，纯路网模式拒绝导航')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                if start_node is None or goal_node is None:
                    self.get_logger().error('无法确定路网节点，纯路网模式拒绝导航')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                if start_node != goal_node:
                    path_nodes, cost = self.graph.plan_path(start_node, goal_node, self.algorithm)
                    if path_nodes is None:
                        self.get_logger().warn(f'路网规划失败: {start_node} -> {goal_node}')
                        self._terminate_goal(goal_handle, False)
                        return NavigateToPose.Result()
                    self.current_path_nodes = path_nodes
                    self.get_logger().info(f'路网路径: {"->".join(path_nodes)}, 总长{cost:.2f}m')
                else:
                    self.current_path_nodes = [start_node]
                    self.get_logger().info('起点和终点为同一路网节点')

                self.current_dense_poses = self._build_network_path(self.current_path_nodes)
                self._publish_route_visualization()

                header = Header()
                header.frame_id = self.map_frame
                header.stamp = self.get_clock().now().to_msg()
                for i in range(len(self.current_path_nodes) - 1):
                    n1 = self.graph.nodes[self.current_path_nodes[i]]
                    n2 = self.graph.nodes[self.current_path_nodes[i + 1]]
                    dx = n2.x - n1.x
                    dy = n2.y - n1.y
                    length = math.sqrt(dx * dx + dy * dy)
                    yaw = math.atan2(dy, dx)
                    num_steps = max(5, int(length / 0.2) + 1)
                    seg_poses = []
                    for j in range(num_steps):
                        t = j / (num_steps - 1) if num_steps > 1 else 0.0
                        x = n1.x + dx * t
                        y = n1.y + dy * t
                        seg_poses.append(self._make_pose(header, x, y, yaw))
                    nav_tasks.append((f'路网_{self.current_path_nodes[i]}->{self.current_path_nodes[i+1]}', 'network', seg_poses))
                # 最后一段的终点朝向使用用户指定的目标朝向
                if nav_tasks and self.current_path_nodes:
                    last_task = nav_tasks[-1]
                    if last_task[1] == 'network' and len(last_task[2]) > 0:
                        last_pose = last_task[2][-1]
                        cy = math.cos(goal_yaw * 0.5)
                        sy = math.sin(goal_yaw * 0.5)
                        last_pose.pose.orientation.x = 0.0
                        last_pose.pose.orientation.y = 0.0
                        last_pose.pose.orientation.z = sy
                        last_pose.pose.orientation.w = cy

            else:
                # ========== 三段式模式（允许路网外） ==========
                # 1) 上路段（自由导航到入口节点）
                if not robot_on_graph:
                    if start_node is None:
                        self.get_logger().warn(f'机器人离路网太远，直接自由导航到终点')
                        result = await self._free_navigate(goal_pose)
                        self._terminate_goal(goal_handle, result)
                        return NavigateToPose.Result()

                    entry_pose = self._node_to_pose(start_node)
                    nav_tasks.append(('上路', 'free', entry_pose))
                    self.get_logger().info(f'上路段: 自由导航到入口节点 {start_node}')

                # 2) 路网段（逐边段导航）
                if start_node is None:
                    start_node = goal_node

                if start_node != goal_node:
                    path_nodes, cost = self.graph.plan_path(start_node, goal_node, self.algorithm)
                    if path_nodes is None:
                        self.get_logger().warn(f'路网规划失败: {start_node} -> {goal_node}')
                        result = await self._free_navigate(goal_pose)
                        self._terminate_goal(goal_handle, result)
                        return NavigateToPose.Result()

                    self.current_path_nodes = path_nodes
                    self.get_logger().info(f'路网路径: {"->".join(path_nodes)}, 总长{cost:.2f}m')

                    self.current_dense_poses = self._build_network_path(path_nodes)

                    header = Header()
                    header.frame_id = self.map_frame
                    header.stamp = self.get_clock().now().to_msg()
                    for i in range(len(path_nodes) - 1):
                        n1 = self.graph.nodes[path_nodes[i]]
                        n2 = self.graph.nodes[path_nodes[i + 1]]
                        dx = n2.x - n1.x
                        dy = n2.y - n1.y
                        length = math.sqrt(dx * dx + dy * dy)
                        yaw = math.atan2(dy, dx)
                        num_steps = max(5, int(length / 0.2) + 1)
                        seg_poses = []
                        for j in range(num_steps):
                            t = j / (num_steps - 1) if num_steps > 1 else 0.0
                            x = n1.x + dx * t
                            y = n1.y + dy * t
                            seg_poses.append(self._make_pose(header, x, y, yaw))
                        nav_tasks.append((f'路网_{path_nodes[i]}->{path_nodes[i+1]}', 'network', seg_poses))

                    # 最后一段的终点朝向：目标在路网上时使用用户指定朝向，
                    # 否则保持边方向（下路段自由导航处理最终朝向）
                    if goal_on_graph and nav_tasks:
                        last_task = nav_tasks[-1]
                        if last_task[1] == 'network' and len(last_task[2]) > 0:
                            last_pose = last_task[2][-1]
                            cy = math.cos(goal_yaw * 0.5)
                            sy = math.sin(goal_yaw * 0.5)
                            last_pose.pose.orientation.x = 0.0
                            last_pose.pose.orientation.y = 0.0
                            last_pose.pose.orientation.z = sy
                            last_pose.pose.orientation.w = cy

                    self._publish_route_visualization()
                else:
                    self.get_logger().info('起点和终点为同一路网节点，跳过路网段')
                    self.current_path_nodes = [start_node] if start_node else []
                    self.current_dense_poses = []
                    self._publish_route_visualization()

                # 3) 下路段（自由导航到最终目标）
                if not goal_on_graph:
                    nav_tasks.append(('下路', 'free', goal_pose))
                    self.get_logger().info(f'下路段: 自由导航到最终目标 ({goal_x:.2f}, {goal_y:.2f})')

            # ---- 分段依次执行 ----
            self._goal_node_id = goal_node
            self._nav_active = True
            self._replan_requested = False
            self._replan_attempts = 0
            total = len(nav_tasks)
            if total == 0:
                self.get_logger().warn('没有可导航的航点')
                self._terminate_goal(goal_handle, True)
                return NavigateToPose.Result()

            self.get_logger().info(f'开始分段导航: 共{total}段')

            seg_index = 0
            while seg_index < len(nav_tasks):
                phase, task_type, data = nav_tasks[seg_index]

                # 抢占检查（段前）：被新目标抢占或被取消 → 自行安全终止
                # 注意：串行化后，这里不要取消 self._active_follow_goal ——
                # 它此刻已是“新导航”的 controller goal，误取消会让新导航瞬间失败。
                if self._is_preempted(goal_handle):
                    self.get_logger().info('导航目标已被抢占或取消，终止执行')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                progress = int(100 * seg_index / total) if total > 1 else 0
                self._send_feedback(goal_handle, f'{phase} {seg_index+1}/{total}', progress)

                # 段间延迟：让 controller_server 清理上一个 goal 的内部状态
                # 避免 "unexpected goal response" 导致后续 get_result_async hang
                if seg_index > 0:
                    await self._sleep(0.5)

                self._current_task_type = task_type
                if task_type == 'free':
                    pose = data
                    px, py = pose.pose.position.x, pose.pose.position.y
                    self.get_logger().info(f'[{seg_index+1}/{total}] {phase}[自由]: ({px:.2f}, {py:.2f})')
                    result = await self._free_navigate(pose)
                else:
                    path = data
                    self.get_logger().info(f'[{seg_index+1}/{total}] {phase}[路网]: {len(path)}个路径点')
                    result = await self._follow_path_with_retry(path, max_retries=2, goal_handle=goal_handle)
                self._current_task_type = ''

                # 障碍触发重规划（仅路网段）：
                #  - 监控器检测到代价地图障碍并置位 _replan_requested（此时跟踪已被取消、result=False）；
                #  - 或 路网段跟踪被控制器中止(result=False)且 _trigger_failure 为真，作为监控失效的兜底。
                # 由 replan_trigger_mode 控制是否启用上述两种触发（'both'/'monitor'/'failure'）。
                # 注意：兜底只在“跟踪失败”时触发，成功到达节点不应触发，否则每段都会误重规划。
                if task_type == 'network' and (self._replan_requested or (self._trigger_failure and not result)):
                    self._replan_requested = False
                    self._replan_attempts += 1
                    if self._replan_attempts > self.replan_max_attempts:
                        self.get_logger().error(
                            f'重规划次数超过上限({self.replan_max_attempts})，终止导航')
                        self._terminate_goal(goal_handle, False)
                        return NavigateToPose.Result()
                    self.get_logger().warn(
                        f'路网段触发重规划(第{self._replan_attempts}次)，尝试改走替代路线')
                    new_tasks = await self._do_replan(goal_handle)
                    if new_tasks is not None:
                        nav_tasks = new_tasks
                        total = len(nav_tasks)
                        seg_index = 0
                        self.get_logger().info(f'重规划成功，从新路线继续（共{total}段）')
                        continue
                    else:
                        self.get_logger().error('重规划失败，无可用替代路线，终止导航')
                        self._terminate_goal(goal_handle, False)
                        return NavigateToPose.Result()

                # 抢占检查（段后）：被新目标抢占或被取消 → 自行安全终止
                if self._is_preempted(goal_handle):
                    self.get_logger().info('导航目标已被抢占或取消，终止执行')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                if not result:
                    self.get_logger().warn(f'第{seg_index+1}段导航失败，中断')
                    self._terminate_goal(goal_handle, False)
                    return NavigateToPose.Result()

                seg_index += 1

            self._send_feedback(goal_handle, '完成', 100)
            self.get_logger().info('全部分段导航完成')
            self._terminate_goal(goal_handle, True)
            return NavigateToPose.Result()

        finally:
            self._nav_active = False
            self._current_task_type = ''
            # 串行收尾：标记本目标结束，若有等待中的新目标则串行驶启动
            with self._goal_lock:
                if self._active_nav is goal_handle:
                    self._active_nav = None
                pending = self._pending_nav
                if pending is not None and pending is not goal_handle:
                    self._pending_nav = None
                    self._active_nav = pending
                    next_goal = pending
                else:
                    next_goal = None
            if next_goal is not None:
                self.get_logger().info('启动被抢占等待的新导航目标')
                if next_goal.is_active:
                    next_goal.execute()
    # ========== 导航核心 ==========

    async def _free_navigate(self, goal_pose):
        """自由导航：planner_server规划路径 -> controller_server跟踪执行"""
        # Step 1: 调用 planner_server 规划路径
        path = await self._compute_path(goal_pose)
        if path is None:
            self.get_logger().error('路径规划失败')
            return False

        self.get_logger().info(f'路径规划成功: {len(path)}个路径点')

        # Step 2: 调用 controller_server 跟踪路径（使用默认DWB控制器）
        return await self._follow_path(path, controller_id='FollowPath')

    async def _compute_path(self, goal_pose):
        """调用 planner_server.ComputePathToPose 获取全局路径"""
        if not self._compute_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('planner_server [compute_path_to_pose] 未就绪')
            return None

        start_pose = self._get_robot_pose()
        if start_pose is None:
            self.get_logger().error('获取机器人位姿失败，无法规划路径')
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose

        send_future = await self._compute_path_client.send_goal_async(goal_msg)
        if not send_future.accepted:
            self.get_logger().error('planner_server 拒绝了规划请求')
            return None

        self._active_compute_goal = send_future
        try:
            result = await send_future.get_result_async()
        except Exception as e:
            self.get_logger().error(f'路径规划异常: {e}')
            if self._active_compute_goal is send_future:
                self._active_compute_goal = None
            return None

        if self._active_compute_goal is send_future:
            self._active_compute_goal = None

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'路径规划返回状态: {result.status}')
            return None

        nav_path = result.result.path
        if not nav_path.poses or len(nav_path.poses) < 2:
            self.get_logger().warn('规划路径为空或只有一个点')
            return None

        return nav_path.poses

    async def _follow_path(self, poses, controller_id=''):
        """调用 controller_server.FollowPath 跟踪执行路径"""
        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('controller_server [follow_path] 未就绪')
            return False

        # 复制路径点列表，避免修改原始数据
        path_poses = list(poses)

        goal_msg = FollowPath.Goal()
        goal_msg.path = NavPath()
        goal_msg.path.header.frame_id = self.map_frame
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg.path.poses = path_poses
        # 指定控制器（路网段用RPP，自由导航用默认DWB）
        if controller_id:
            goal_msg.controller_id = controller_id
            self.get_logger().info(f'使用控制器: {controller_id}')

        send_future = await self._follow_path_client.send_goal_async(goal_msg)
        if not send_future.accepted:
            self.get_logger().error('controller_server 拒绝了跟踪请求')
            return False

        self._active_follow_goal = send_future
        try:
            result = await send_future.get_result_async()
        except Exception as e:
            self.get_logger().error(f'路径跟踪异常: {e}')
            if self._active_follow_goal is send_future:
                self._active_follow_goal = None
            return False

        if self._active_follow_goal is send_future:
            self._active_follow_goal = None

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'路径跟踪返回状态: {result.status}')
            return False

        return True

    async def _follow_network_path(self, poses):
        """路网段导航：使用RPP控制器（更适合路径跟踪，不会因调头卡住）"""
        if not poses or len(poses) < 2:
            self.get_logger().warn('路网路径点不足，跳过')
            return True
        return await self._follow_path(poses, controller_id=self.network_controller_id)

    async def _sleep(self, seconds):
        """段间延迟（阻塞事件循环，影响 0.5s 内新目标响应）"""
        import time
        time.sleep(seconds)

    async def _follow_path_with_retry(self, poses, max_retries=2, goal_handle=None):
        """调用 FollowPath（RPP控制器），失败后自动重试"""
        for attempt in range(max_retries + 1):
            # 抢占检查：如果目标已被新目标抢占或取消，停止重试
            if goal_handle is not None and self._is_preempted(goal_handle):
                self.get_logger().info('导航目标已被抢占，停止重试')
                return False
            # 障碍重规划请求：立即停止当前路网跟踪，交还主循环处理
            if self._replan_requested:
                self.get_logger().info('检测到重规划请求，停止当前路网跟踪')
                return False
            if attempt > 0:
                self.get_logger().warn(
                    f'路径跟踪失败，重试 {attempt}/{max_retries}')
            result = await self._follow_path(poses, controller_id=self.network_controller_id)
            if result:
                return True
        return False

    def _build_network_path(self, node_path):
        """将节点路径构建为密集路径点列表（供controller_server跟踪）"""
        if not node_path or len(node_path) < 1:
            return []

        poses = []
        self._dense_node_ids = []
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()

        for i in range(len(node_path) - 1):
            n1 = self.graph.nodes[node_path[i]]
            n2 = self.graph.nodes[node_path[i + 1]]

            dx = n2.x - n1.x
            dy = n2.y - n1.y
            length = math.sqrt(dx * dx + dy * dy)
            yaw = math.atan2(dy, dx)

            if length < 1e-6:
                continue

            # 密集化：每隔 dense_step 米一个点
            num_steps = max(1, int(length / self.dense_step))
            for j in range(num_steps):
                t = j / num_steps
                x = n1.x + dx * t
                y = n1.y + dy * t
                poses.append(self._make_pose(header, x, y, yaw))
                self._dense_node_ids.append(node_path[i])

        # 确保最后一个节点也在路径中
        last = self.graph.nodes[node_path[-1]]
        if len(node_path) >= 2:
            prev = self.graph.nodes[node_path[-2]]
            yaw = math.atan2(last.y - prev.y, last.x - prev.x)
        else:
            yaw = 0.0
        poses.append(self._make_pose(header, last.x, last.y, yaw))
        self._dense_node_ids.append(node_path[-1])

        return poses

    # ========== 代价地图与障碍重规划 ==========

    def _costmap_callback(self, msg):
        """缓存全局代价地图（nav2_msgs/Costmap）"""
        self._costmap_meta = msg.metadata
        self._costmap_data = msg.data
        if not self._costmap_received:
            self._costmap_received = True
            self.get_logger().info(
                f'已收到代价地图: {self.costmap_topic} '
                f'({msg.metadata.size_x}x{msg.metadata.size_y}, '
                f'res={msg.metadata.resolution:.3f}m)'
            )

    def _world_to_map(self, wx, wy):
        """世界坐标 -> 代价地图栅格坐标"""
        meta = self._costmap_meta
        mx = int((wx - meta.origin.position.x) / meta.resolution)
        my = int((wy - meta.origin.position.y) / meta.resolution)
        return mx, my

    def _get_cost(self, wx, wy):
        """查询世界坐标处的代价值；越界或未知返回 -1。

        注意 nav2_msgs/Costmap.data 为 int8 数组，需用 & 0xFF 还原成 0~255；
        255 为 NO_INFORMATION（未知），按非障碍处理。
        """
        if self._costmap_data is None or self._costmap_meta is None:
            return -1
        meta = self._costmap_meta
        mx, my = self._world_to_map(wx, wy)
        sx = int(meta.size_x)
        sy = int(meta.size_y)
        if mx < 0 or my < 0 or mx >= sx or my >= sy:
            return -1
        idx = my * sx + mx
        if idx < 0 or idx >= len(self._costmap_data):
            return -1
        return self._costmap_data[idx] & 0xFF

    def _monitor_callback(self):
        """定时检测路网前方路径上的障碍，触发重规划"""
        if not (self._trigger_monitor and self.loaded and self._nav_active
                and self._current_task_type == 'network' and not self._replan_requested):
            return
        if self._costmap_data is None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._costmap_warn_ts > 10.0:
                self.get_logger().warn(
                    f'尚未收到代价地图({self.costmap_topic})，无法进行障碍检测；'
                    f'请确认该话题存在且消息类型为 nav2_msgs/Costmap '
                    f'（Nav2 Humble 中对应 /global_costmap/costmap_raw）。')
                self._costmap_warn_ts = now
            return
        if not self.current_dense_poses or not self._dense_node_ids:
            return
        robot = self._get_robot_pose()
        if robot is None:
            return
        rx = robot.pose.position.x
        ry = robot.pose.position.y
        # 从机器人当前位置之后的路径点开始检查（已走过的点忽略）
        start_idx = self._nearest_index_in_path(rx, ry)
        blocked_count = 0
        first_blocked_idx = None
        for k in range(start_idx, len(self.current_dense_poses)):
            p = self.current_dense_poses[k].pose.position
            # 只看机器人前方 cost_lookahead 米内的路径点
            if math.hypot(p.x - rx, p.y - ry) > self.cost_lookahead:
                break
            c = self._get_cost(p.x, p.y)
            if c < 0 or c == 255:
                continue
            if c > self.cost_threshold:
                blocked_count += 1
                if first_blocked_idx is None:
                    first_blocked_idx = k
            if blocked_count >= self.path_obstacles_threshold:
                self._blocked_seg_start = self._dense_node_ids[first_blocked_idx]
                # _dense_node_ids 记录每个密集点所属边的“起点节点 id”。
                # 障碍可能位于某条边中间，因此向前扫描找到该边终点（节点 id 变化处）。
                edge_end = self._dense_node_ids[-1]
                for m in range(first_blocked_idx + 1, len(self._dense_node_ids)):
                    if self._dense_node_ids[m] != self._blocked_seg_start:
                        edge_end = self._dense_node_ids[m]
                        break
                self._blocked_seg_next = edge_end
                self._replan_requested = True
                self.get_logger().warn(
                    f'检测到路网前方障碍(代价>{self.cost_threshold}的点≥'
                    f'{self.path_obstacles_threshold})，触发重规划，'
                    f'堵塞边 {self._blocked_seg_start}->{self._blocked_seg_next}')
                if self._active_follow_goal is not None:
                    self._active_follow_goal.cancel_goal_async()
                return

    def _nearest_index_in_path(self, x, y):
        """返回路径点中离 (x,y) 最近的索引"""
        best = 0
        best_d = float('inf')
        for i, p in enumerate(self.current_dense_poses):
            dx = p.pose.position.x - x
            dy = p.pose.position.y - y
            d = dx * dx + dy * dy
            if d < best_d:
                best_d = d
                best = i
        return best

    def _find_edge_id(self, start, end):
        """查找两节点间的边 id（无向，匹配邻接表任一方向）"""
        for neighbor, edge_id, _w in self.graph.adjacency.get(start, []):
            if neighbor == end:
                return edge_id
        for neighbor, edge_id, _w in self.graph.adjacency.get(end, []):
            if neighbor == start:
                return edge_id
        return None

    async def _do_replan(self, goal_handle):
        """障碍重规划：以最近路网点为锚点，临时摘除堵塞边后重规划剩余路线。

        返回新的 nav_tasks 列表，失败返回 None。
        """
        robot = self._get_robot_pose()
        if robot is None or self._goal_node_id is None:
            self.get_logger().warn('重规划失败：无法获取机器人位姿或目标节点')
            return None
        rx = robot.pose.position.x
        ry = robot.pose.position.y
        anchor = self.graph.find_nearest_node(rx, ry, self.snap_distance)
        if anchor is None:
            anchor = self.graph.find_nearest_node(rx, ry, float('inf'))
        if anchor is None:
            self.get_logger().warn('重规划失败：找不到锚点节点')
            return None

        # 若监控器未指定堵塞边（如由控制器中止兜底触发），
        # 则依据机器人最近节点(anchor)在“当前路线”上的位置，推断其前方正在通行的边。
        # 注意：必须取“从 anchor 出发的下游边”，避免使用 _nearest_index_in_path 落在已走过的点上
        # 导致摘除“身后边”后重规划仍走原路（误判无替代路线）。
        if self._blocked_seg_next is None and self.current_path_nodes:
            try:
                aidx = self.current_path_nodes.index(anchor)
                if aidx + 1 < len(self.current_path_nodes):
                    self._blocked_seg_start = anchor
                    self._blocked_seg_next = self.current_path_nodes[aidx + 1]
                elif aidx - 1 >= 0:
                    self._blocked_seg_start = self.current_path_nodes[aidx - 1]
                    self._blocked_seg_next = anchor
            except ValueError:
                pass

        # 临时摘除堵塞边，逼出替代路线（规划后恢复，不影响后续导航）
        blocked_edge = None
        if self._blocked_seg_next is not None:
            blocked_edge = self._find_edge_id(self._blocked_seg_start, self._blocked_seg_next)
        saved_edge = None
        if blocked_edge is not None and blocked_edge in self.graph.edges:
            e = self.graph.edges[blocked_edge]
            saved_edge = (blocked_edge, e.start_node, e.end_node, e.weight,
                          e.bidirectional, e.name, dict(e.metadata))
            self.graph.remove_edge(blocked_edge)

        new_nodes, _cost = self.graph.plan_path(anchor, self._goal_node_id, self.algorithm)

        # 恢复被摘除的边
        if saved_edge is not None:
            self.graph.add_edge(*saved_edge[:6])
            self.graph.edges[saved_edge[0]].metadata = saved_edge[6]

        if new_nodes is None or len(new_nodes) < 1:
            self.get_logger().warn('重规划失败：无替代路线')
            return None

        # 若重规划结果与当前剩余路线一致（同一锚点出发走原路），说明无替代路线，
        # 直接判定失败，避免“摘除边→恢复原路→再触发”的无限循环
        try:
            idx = self.current_path_nodes.index(anchor)
            if new_nodes == self.current_path_nodes[idx:]:
                self.get_logger().warn('重规划结果与原路线一致，无替代路线')
                return None
        except ValueError:
            pass

        # 重建密集路径（起点=anchor；同时刷新 _dense_node_ids）
        new_dense = self._build_network_path(new_nodes)
        if not new_dense:
            return None

        tasks = []
        if self.replan_reverse_via_freenav:
            # 自由导航接驳回锚点；网络段从锚点开始
            anchor_pose = self._node_to_pose(anchor)
            tasks.append(('重规划接驳', 'free', anchor_pose))
        else:
            # 不接驳：网络路径直接从机器人当前位姿接续（可能需倒车）
            new_dense.insert(0, robot)
            self._dense_node_ids.insert(0, anchor)

        # 终点朝向使用用户设定的目标朝向
        cy = math.cos(self._goal_yaw * 0.5)
        sy = math.sin(self._goal_yaw * 0.5)
        last = new_dense[-1]
        last.pose.orientation.x = 0.0
        last.pose.orientation.y = 0.0
        last.pose.orientation.z = sy
        last.pose.orientation.w = cy

        tasks.append(('重规划路网', 'network', new_dense))

        self.current_path_nodes = new_nodes
        self.current_dense_poses = new_dense
        self.get_logger().info(f'重规划路线: {"->".join(new_nodes)}')
        return tasks

    # ========== 辅助方法 ==========

    def _densify_node_path(self, node_path):
        """将节点路径密集化（仅用于可视化）—— 复用 _build_network_path"""
        return self._build_network_path(node_path)

    def _pose_to_yaw(self, pose):
        """从 PoseStamped 提取 yaw 朝向"""
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _make_pose(self, header, x, y, yaw):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sy
        pose.pose.orientation.w = cy
        return pose

    def _node_to_pose(self, node_id):
        node = self.graph.nodes[node_id]
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()
        return self._make_pose(header, node.x, node.y, 0.0)

    def _get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF2获取位姿失败: {e}')
            return None

    def _is_pose_on_graph(self, x, y):
        return self.graph.is_on_graph(x, y, self.on_graph_threshold)

    # ========== 路网加载 ==========

    def _load_route_graph(self):
        route_file = self.get_parameter('route_file').value
        if route_file:
            # 支持 ~ / $HOME 等写法（bash 在 --ros-args 赋值中不会展开 ~）
            route_file = os.path.expanduser(os.path.expandvars(route_file))
        if not route_file or not os.path.exists(route_file):
            # 兜底：优先用 ament 包共享目录，
            # colcon 安装后 routes/warehouse_routes.geojson 就在这里。
            possible = []
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory('route_planner')
                possible.append(
                    os.path.join(share_dir, 'routes', 'warehouse_routes.geojson'))
            except Exception:
                pass
            # 旧兜底（源码/未安装场景）作为补充
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            possible.append(os.path.join(pkg_dir, 'routes', 'warehouse_routes.geojson'))
            possible.append(os.path.join(pkg_dir, '..', 'routes', 'warehouse_routes.geojson'))
            for p in possible:
                if os.path.exists(p):
                    route_file = p
                    break

        if not route_file or not os.path.exists(route_file):
            self.get_logger().error(f'未找到路网文件: {route_file}')
            return False

        self.get_logger().info(f'加载路网: {route_file}')
        if not self.parser.parse_file(route_file):
            self.get_logger().error('路网解析失败')
            return False

        valid, errors = self.parser.validate()
        if not valid:
            for err in errors:
                self.get_logger().warn(f'路网验证: {err}')

        self.graph.clear()
        for nid, ndata in self.parser.get_nodes().items():
            self.graph.add_node(nid, ndata['x'], ndata['y'], ndata['z'],
                                ndata.get('name', nid), ndata.get('properties', {}))
        for eid, edata in self.parser.get_edges().items():
            self.graph.add_edge(eid, edata['start_node'], edata['end_node'],
                                edata['weight'], edata.get('bidirectional', True),
                                edata.get('name', eid), edata.get('properties', {}))

        self.loaded = True
        stats = self.graph.get_statistics()
        self.get_logger().info(
            f'路网加载成功: {stats["node_count"]}节点, {stats["edge_count"]}边, '
            f'总长{stats["total_edge_length"]:.1f}m')
        if stats['one_way_edges']:
            self.get_logger().info(f'单行线(单向通行): {stats["one_way_edges"]}条')
        self._publish_route_visualization()
        return True

    # ========== 可视化 ==========

    def _timer_callback(self):
        if self.loaded:
            self._publish_route_visualization()

    def _publish_route_visualization(self):
        if not self.loaded:
            return

        marker_array = MarkerArray()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        # 节点
        node_marker = Marker()
        node_marker.header = header
        node_marker.ns = 'route_nodes'
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = self.marker_scale * 2.0
        node_marker.scale.y = self.marker_scale * 2.0
        node_marker.scale.z = self.marker_scale * 2.0
        node_marker.color = ColorRGBA(r=0.9, g=0.2, b=0.2, a=0.9)

        # 双向边
        edge_marker = Marker()
        edge_marker.header = header
        edge_marker.ns = 'route_edges'
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = self.marker_scale * 0.5
        edge_marker.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.8)

        # 单行线（单向通行）边：橙色区分
        oneway_edge_marker = Marker()
        oneway_edge_marker.header = header
        oneway_edge_marker.ns = 'oneway_edges'
        oneway_edge_marker.id = 3
        oneway_edge_marker.type = Marker.LINE_LIST
        oneway_edge_marker.action = Marker.ADD
        oneway_edge_marker.scale.x = self.marker_scale * 0.5
        oneway_edge_marker.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.9)

        # 规划路径
        path_marker = Marker()
        path_marker.header = header
        path_marker.ns = 'planned_route'
        path_marker.id = 2
        path_marker.type = Marker.LINE_LIST
        path_marker.action = Marker.ADD
        path_marker.scale.x = self.marker_scale * 1.0
        path_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)

        for nid, node in self.graph.nodes.items():
            node_marker.points.append(Point(x=node.x, y=node.y, z=node.z + 0.05))
            tm = Marker()
            tm.header = header
            tm.ns = 'route_labels'
            tm.id = abs(hash(nid)) % 2147483647
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = node.x
            tm.pose.position.y = node.y
            tm.pose.position.z = node.z + 0.3
            tm.scale.z = self.marker_scale * 1.2
            tm.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            tm.text = nid
            marker_array.markers.append(tm)

        for edge in self.graph.edges.values():
            s = self.graph.nodes[edge.start_node]
            e = self.graph.nodes[edge.end_node]
            if edge.bidirectional:
                edge_marker.points.append(Point(x=s.x, y=s.y, z=s.z + 0.02))
                edge_marker.points.append(Point(x=e.x, y=e.y, z=e.z + 0.02))
            else:
                # 单行线：橙色边 + 末端方向箭头（指向允许通行方向）
                oneway_edge_marker.points.append(Point(x=s.x, y=s.y, z=s.z + 0.02))
                oneway_edge_marker.points.append(Point(x=e.x, y=e.y, z=e.z + 0.02))
                dx = e.x - s.x
                dy = e.y - s.y
                length = math.sqrt(dx * dx + dy * dy)
                if length > 1e-6:
                    ux, uy = dx / length, dy / length
                    yaw = math.atan2(dy, dx)
                    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
                    # 沿边均匀放置小箭头：短边1个（中点），长边2~3个
                    arrow_span = min(length * 0.35, 1.5)
                    num_arrows = max(1, min(3, int(length / arrow_span)))
                    for ai in range(num_arrows):
                        t = (ai + 0.65) / (num_arrows + 0.3)  # 偏向末端
                        ax = s.x + dx * t
                        ay = s.y + dy * t
                        az = s.z + (e.z - s.z) * t + 0.04
                        am = Marker()
                        am.header = header
                        am.ns = 'oneway_arrows'
                        am.id = hash(f'{edge.id}_{ai}') % 2147483647
                        am.type = Marker.ARROW
                        am.action = Marker.ADD
                        am.pose.position.x = ax
                        am.pose.position.y = ay
                        am.pose.position.z = az
                        am.pose.orientation.x = 0.0
                        am.pose.orientation.y = 0.0
                        am.pose.orientation.z = sy
                        am.pose.orientation.w = cy
                        am.scale.x = self.marker_scale * 0.7
                        am.scale.y = self.marker_scale * 0.25
                        am.scale.z = self.marker_scale * 0.2
                        am.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.95)
                        marker_array.markers.append(am)

        if self.current_path_nodes:
            for i in range(len(self.current_path_nodes) - 1):
                n1 = self.graph.nodes[self.current_path_nodes[i]]
                n2 = self.graph.nodes[self.current_path_nodes[i + 1]]
                path_marker.points.append(Point(x=n1.x, y=n1.y, z=n1.z + 0.08))
                path_marker.points.append(Point(x=n2.x, y=n2.y, z=n2.z + 0.08))

        marker_array.markers.append(node_marker)
        marker_array.markers.append(edge_marker)
        marker_array.markers.append(oneway_edge_marker)
        if self.current_path_nodes:
            marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)

        if self.current_dense_poses:
            path_msg = NavPath()
            path_msg.header = header
            path_msg.poses = self.current_dense_poses
            self.route_path_pub.publish(path_msg)

    def _send_feedback(self, goal_handle, text, progress):
        # 仅对仍活跃的目标发布反馈，避免向已终止/被抢占目标推送 → RViz 空指针风险
        if not goal_handle.is_active:
            return
        feedback = NavigateToPose.Feedback()
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = RouteNavigatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
