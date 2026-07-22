# route_planner — 路网约束导航功能包

Nav2 附加导航层，让机器人沿预定义路网行驶。版本 2.0.0 | ROS2 Humble | Apache-2.0

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

**方式一：接入已有 Nav2 栈（推荐）**

先启动 Nav2 栈（不启动 `bt_navigator`），然后：

```bash
ros2 launch route_planner route_planner.launch.py route_file:=/path/to/routes.geojson
```

**方式二：使用本包自带 Nav2 全栈**

```bash
# 先启动机器人本体（URDF+激光等）
ros2 launch route_planner gazebo_route_navigation.launch.py
```

### Nav2 配置要求

1. `controller_server` 需含 `FollowPath`（DWB）和 `FollowPathRPP`（RPP）两个插件
2. 全局代价地图需设 `always_send_full_costmap: true`，配 `obstacle_layer` 接激光
3. 节点订阅 `/global_costmap/costmap_raw`（非 `/global_costmap/costmap`）

### 验证

```bash
ros2 node list  # 应看到 route_navigator_node
ros2 action list | grep -E "compute_path_to_pose|follow_path"
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

## 常见问题

**节点起不来**：确认 Nav2 栈已运行且 action 可用（不启动 `bt_navigator`）

**重规划不触发**：检查日志是否收到代价地图，确认 `obstacle_layer` 配置，或改用 `failure` 模式

**禁用路网约束**：设 `enabled: false`

**坐标系不同**：修改 `robot_base_frame` 参数

## 许可证

Apache-2.0
