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

```bash
ros2 launch route_planner route_planner.launch.py \
  route_file:=/path/to/routes.geojson \
  algorithm:=astar
```

此 launch 仅注册路网节点 `route_planner_node`，不自带 Nav2。

**方式二：Gazebo 仿真（自带 Nav2 全栈）**

`gazebo_route_navigation.launch.py` 会自动拉起完整 Nav2 栈（map_server / amcl / planner_server / controller_server 等，不含 bt_navigator）+ 路网节点 `route_navigator_node`，默认 `use_sim_time:=true`：

```bash
# 先启动 Gazebo 仿真中的机器人本体（URDF + 激光 /scan + 里程计 /odom），例如：
#   ros2 launch <你的机器人包> gazebo_sim.launch.py
ros2 launch route_planner gazebo_route_navigation.launch.py \
  route_file:=/path/to/routes.geojson \
  allow_offroad:=true
```

默认使用包内 `maps/sim_room.yaml` 地图、`config/nav2_params_sim.yaml` 参数、`routes/warehouse_routes.geojson` 路网。

**方式三：实车导航（mycar 整车）**

`route_navigation.launch.py` 面向实车，会拉起底盘驱动、激光雷达、摄像头以及完整 Nav2 栈 + 路网节点：

```bash
ros2 launch route_planner route_navigation.launch.py \
  route_file:=/path/to/routes.geojson \
  allow_offroad:=true
```

依赖 `mycar_nav2`、`turn_on_wheeltec_robot` 两个配套包（含底盘驱动与实车参数），仅 mycar 整车使用。

### Nav2 配置要求

1. `controller_server` 需同时加载 `FollowPath`（DWB，上下路用）和 `FollowPathRPP`（RPP，路网段用）两个插件
2. 全局代价地图需设 `always_send_full_costmap: true`，并配 `obstacle_layer` 接 `/scan` 激光
3. 路网节点订阅 `/global_costmap/costmap_raw`（nav2_msgs/Costmap，0~255），非 `/global_costmap/costmap`（OccupancyGrid，0~100）

### 启动后验证

```bash
# 方式一：应看到 route_planner_node
# 方式二：应看到 route_navigator_node + map_server + amcl + planner_server + controller_server
ros2 node list

# 两个 action 必须存在
ros2 action list | grep -E "compute_path_to_pose|follow_path"

# RViz 用 "2D Goal Pose" 点目标，机器人应沿路网行驶
```

## 使用

RViz 中用 **"2D Goal Pose"** 点目标位置并拖箭头设朝向。

- 纯路网模式（`allow_offroad:=false`）：路网外目标被拒绝
- 三段式模式（`allow_offroad:=true`）：允许自由上下路

## 路网编辑

```bash
python3 src/route_planner/route_editor.py \
  --map src/route_planner/maps/sim_room.pgm \
  --yaml src/route_planner/maps/sim_room.yaml
```

操作：点击添加节点 | 依次点两节点添加边 | 拖拽移动 | Delete 删除 | 双击编辑名称 | Ctrl+S 保存

## 参数配置

**基础参数**（`config/route_planner.yaml`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `route_file` | `""` | GeoJSON 路径，留空自动查找 |
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
| `replan_enabled` | `true` | 总开关 |
| `replan_trigger_mode` | `both` | 触发模式：both/monitor/failure |
| `cost_threshold` | `60` | 单点代价阈值（0~255） |
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
