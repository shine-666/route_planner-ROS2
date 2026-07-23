# route_planner — 路网约束导航功能包

Nav2 附加导航层，让机器人沿预定义路网行驶  版本 2.0.0 | ROS2 Humble | Apache-2.0

## 效果演示

【ROS2路网规划导航-哔哩哔哩】 https://b23.tv/qYzpKwO

## 快速开始

### 依赖安装

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-regulated-pure-pursuit
pip3 install Pillow  # 仅路网编辑器需要
```

### 编译

```bash
cd <工作区>
colcon build --packages-select route_planner
source install/setup.bash
```

### 启动方式

**方式一：仅路网节点（需自备 Nav2 栈）**

先启动已有 Nav2 栈（map_server + amcl + planner_server + controller_server，**不启动 bt_navigator**），再启动本包：

具体可参考 `gazebo_route_navigation.launch.py` 和 `route_navigation.launch.py`

```bash
ros2 launch route_planner route_planner.launch.py \
  route_file:=/path/to/routes.geojson \
  algorithm:=astar
```

此 launch 仅注册路网节点 `route_planner_node`

**方式二：Gazebo 仿真**

推荐使用 [fishros/ros2_patrol_robot](https://github.com/fishros/ros2_patrol_robot) 项目中的 `fishbot_description` 包进行仿真，已验证可用。

**步骤 1**：克隆 ros2_patrol_robot 仿真项目并编译

```bash
cd <你的工作区>/src
git clone https://github.com/fishros/ros2_patrol_robot.git
cd <你的工作区>
colcon build
source install/setup.bash
```

**步骤 2**：启动 Gazebo 仿真

```bash
ros2 launch fishbot_description gazebo.launch.py
```

该命令会启动 Gazebo 世界、FishBot 机器人模型（URDF）、激光雷达（`/scan`）和里程计（`/odom`）。

**步骤 3**：启动路网导航

`gazebo_route_navigation.launch.py` 会自动拉起完整 Nav2 栈 + 路网节点 `route_navigator_node`，默认 `use_sim_time:=true`：

```bash
ros2 launch route_planner gazebo_route_navigation.launch.py \
  route_file:=/path/to/routes.geojson \
  allow_offroad:=true
```

默认使用包内 `maps/sim_room.yaml` 地图、`config/nav2_params_sim.yaml` 参数、`routes/warehouse_routes.geojson` 路网。

> 若使用其他机器人平台，只需保证仿真节点发布 `/scan`、`/odom` 和 `map → odom → base_link` TF 链即可。

**方式三：实车导航**

`route_navigation.launch.py` 面向实车，会拉起底盘驱动、激光雷达、摄像头以及完整 Nav2 栈 + 路网节点：

```bash
ros2 launch route_planner route_navigation.launch.py \
  route_file:=/path/to/routes.geojson \
  allow_offroad:=true
```

注意：依赖底盘等硬件驱动

### Nav2 配置要求

1. `controller_server` 需同时加载 `FollowPath`（DWB，上下路用）和 `FollowPathRPP`（RPP，路网段用）两个插件
2. 全局代价地图需设 `always_send_full_costmap: true`，并配 `obstacle_layer` 接 `/scan` 激光
3. 路网节点订阅 `/global_costmap/costmap_raw`（nav2_msgs/Costmap，0~255），非 `/global_costmap/costmap`（OccupancyGrid，0~100）

### 启动后验证

```bash
ros2 node list
# 应看到 route_planner_node
```

## 使用

RViz 中添加 `route_graph_markers`（MarkerArray）话题和 2D Goal Pose 工具

使用 **"2D Goal Pose"** 点目标位置并拖箭头设朝向。

- 纯路网模式（`allow_offroad:=false`）：路网外目标被拒绝
- 三段式模式（`allow_offroad:=true`）：允许自由上下路

## 路网编辑

```bash
python3 src/route_planner/route_editor.py \
  --map src/route_planner/maps/sim_room.pgm \
  --yaml src/route_planner/maps/sim_room.yaml
```
或者
```bash
python3 src/route_planner/route_editor.py
```
可在图形界面打开地图和路网文件

操作：点击添加节点 | 依次点两节点添加边 | 拖拽移动 | Delete 删除 | 双击编辑名称 | Ctrl+S 保存

## 参数配置

**基础参数**（`config/route_planner.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `route_file` | `""` | GeoJSON 路径 |
| `map_frame` | `map` | 地图坐标系 |
| `robot_base_frame` | `base_link` | 机器人 base 坐标系 |
| `algorithm` | `astar` | 规划算法（astar/dijkstra） |
| `snap_distance` | `3.0` | 最大吸附距离（米） |
| `on_graph_threshold` | `0.5` | "在路网上"判断阈值（米） |
| `dense_step` | `0.2` | 路网密集化步长（米） |
| `allow_offroad` | `false` | 允许路网外自由导航 |
| `enabled` | `true` | 启用路网约束 |
| `network_controller_id` | `FollowPathRPP` | 路网段控制器插件名 |

**重规划参数**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `replan_enabled` | `true` | 是否启用重规划功能 |
| `replan_trigger_mode` | `both` | 触发模式：both/monitor/failure |
| `cost_threshold` | `60` | 单点代价判定阈值 |
| `path_obstacles_threshold` | `2` | 前方高代价点数触发阈值 |
| `cost_lookahead` | `1.5` | 前方检测距离（米） |
| `monitor_hz` | `2.0` | 监控频率（Hz） |
| `costmap_topic` | `/global_costmap/costmap_raw` | 代价地图话题 |
| `replan_max_attempts` | `5` | 最大重规划次数 |

## 接口

**Action**
- `/navigate_to_pose`（Server）— 接收目标
- `compute_path_to_pose`（Client）— 调用 planner_server
- `follow_path`（Client）— 调用 controller_server

**话题**
- 订阅：`/goal_pose`、`/route_planner/goal`（PoseStamped）
- 订阅：`/global_costmap/costmap_raw`（仅 monitor 模式）
- 发布：`/route_path`（Path）、`/route_graph_markers`（MarkerArray）

## GeoJSON 格式

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": { "type": "Point", "coordinates": [2.0, 3.0, 0.0] },
      "properties": { "id": "1", "type": "node", "name": "入库点" }
    },
    {
      "type": "Feature",
      "geometry": { "type": "LineString", "coordinates": [[2.0, 3.0, 0.0], [5.0, 3.0, 0.0]] },
      "properties": { "id": "e1", "type": "edge", "start_node": "1", "end_node": "2", "bidirectional": true }
    }
  ]
}
```

coordinates 为 `[x, y, z]`（米），边 `bidirectional: true` 表示双向通行。

## 许可证

Apache-2.0
