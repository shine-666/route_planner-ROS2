# route_planner — 路网约束导航功能包

`route_planner` 是 **Nav2 的一个附加导航层**：机器人沿你预先绘制的路网行驶，而不是自由乱走。它**不替代、不绑定任何特定机器人驱动**，只要在运行着标准 Nav2 栈的 ROS2 环境里，就能叠加路网约束导航。

- 版本：2.0.0 ｜ 平台：ROS2 **Humble** ｜ 许可证：Apache-2.0
- 节点入口：`route_navigator_node`（一个 Python 进程，内含全部逻辑）

---

## ⚠️ 重要前提（如果你**只有这一个功能包**）

本包只提供"路网约束导航逻辑"，**不含机器人驱动，也不含完整 Nav2 启动**。请按你手里的东西选启动方式：

| 启动文件 | 依赖什么 | 能否**独立**用 |
|---|---|---|
| `route_planner.launch.py` | 仅本包；**需你另起 Nav2 栈** | ✅ 能（推荐给已有机器人的开发者） |
| `gazebo_route_navigation.launch.py` | 仅本包内 `config/`+`maps/` + 系统包 `nav2_bringup`；**需你另起机器人本体（Gazebo/URDF）** | ✅ 能（自带完整 Nav2 栈） |
| `route_navigation.launch.py` | 依赖 `mycar_nav2`、`turn_on_wheeltec_robot` 等其它包 | ❌ 不能（仅原 mycar 整车用） |

> **`package.xml` 的清理提示**：当前 `package.xml` 里写了
> `<exec_depend>mycar_nav2</exec_depend>` 和 `<exec_depend>turn_on_wheeltec_robot</exec_depend>`，
> 这两个仅为上面"整车 launch"服务。**独立部署（只用本包）时它们用不到**，可放心删掉，
> 不影响 `route_planner` 任何功能，也能避免 `rosdep` 去拉你并不需要的包。

---

## 主要功能

- **路网约束导航**：机器人严格沿预定义路网行驶，不到处乱走。
- **GeoJSON 路网**：用标准 GeoJSON 定义节点与边，附带可视化编辑器，在地图底图上画即可。
- **自动上下路 + 吸附**：不在路网上时自动自由导航到最近入口；目标在路网上则全程沿路网。
- **标准 RViz 交互**：用 RViz 的 **"2D Goal Pose"** 下发目标，和原生 Nav2 习惯一致。
- **障碍物动态重规划**：路网段中前方遇障，自动回退到最近路网点绕行（两种触发方式可选）。
- **可关闭的路网约束**：设 `enabled: false` 即退化为标准 Nav2 自由导航。
- **纯路网 / 三段式 两种模式**：`allow_offroad=false` 禁走路网外；`=true` 允许自由上下路。

---

## 环境依赖

**ROS2 系统包（apt 安装）：**

```bash
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-nav2-regulated-pure-pursuit
```

- `nav2-bringup`：提供 Nav2 各底层节点（map_server / amcl / planner_server / controller_server 等）；
- `nav2-regulated-pure-pursuit`：路网段控制器 RPP 插件。

> 节点运行还实际用到 `tf2_ros`、`ament_index_python`，但当前 `package.xml` 未显式声明，靠传递依赖隐式拉入。
> 若你在干净环境编译报 `ModuleNotFoundError: No module named 'tf2_ros'`（或 `ament_index_python`），
> 请在 `package.xml` 补 `<depend>tf2_ros</depend>` 与 `<depend>ament_index_python</depend>` 后重新编译。

**Python 依赖（仅 `route_editor.py` 路网编辑器需要）：**

```bash
pip3 install Pillow
```

**编译：**

```bash
cd <你的工作区>
colcon build --packages-select route_planner
source install/setup.bash
```

---

## 部署（核心）

`route_planner` 必须在 **Nav2 栈已运行** 的环境里工作。Nav2 负责地图、定位、全局规划、局部控制；
本节点只在这些服务之上叠加"路网约束"。下面两种部署方式二选一。

### 方式一：接入你已有的 Nav2 栈（推荐给已有机器人的开发者）

1. 先把你自己的 Nav2 栈跑起来（map_server + amcl + planner_server + controller_server + 必要的 lifecycle_manager 等），并保证三个 action 可用：
   - `compute_path_to_pose`（planner_server 提供）
   - `follow_path`（controller_server 提供）
   - **不要**启动 `bt_navigator`（它会抢注 `/navigate_to_pose`，与本节点冲突）
2. 启动路网节点（它自己注册 `/navigate_to_pose` 作为 ActionServer）：

   ```bash
   ros2 launch route_planner route_planner.launch.py \
     route_file:=/path/to/your_routes.geojson
   ```

### 方式二：用本包自带的 Nav2 全栈（仿真 / 快速验证）

`gazebo_route_navigation.launch.py` 会一并启动一份**完整 Nav2 栈**（配置来自本包 `config/nav2_params_sim.yaml`、地图来自 `maps/sim_room.yaml`），**不依赖 mycar 其它包**。

```bash
# 另开终端先启动你的机器人本体（URDF + 激光等），例如在 Gazebo 中：
#   ros2 launch <你的机器人包> gazebo_sim.launch.py
# 再启动本包（自带 Nav2 + 路网节点）：
ros2 launch route_planner gazebo_route_navigation.launch.py
```

> 本包**不含机器人模型/驱动**，方式二仍需你自行把机器人本体（含激光话题 `/scan`、里程计 `/odom` 等）跑起来。

### Nav2 侧必须满足的条件

无论用哪种方式，Nav2 都要满足以下几点，否则路网节点无法正常工作（以 `config/nav2_params_sim.yaml` 为准，可直接复制作模板）：

1. **`controller_server` 必须同时具备两个控制器插件**：
   - `FollowPath`：默认 DWB 局部规划器，用于上下路自由导航；
   - `FollowPathRPP`：必须是 **Regulated Pure Pursuit** 插件，用于路网段跟踪。
   - 插件名务必与此一致（路网段控制器名由参数 `network_controller_id` 指定，默认 `FollowPathRPP`）。
2. **全局代价地图（仅 `replan_trigger_mode` 含 `monitor` 时需要）**：
   - `global_costmap` 设 `always_send_full_costmap: true`，否则节点收不到完整代价数据；
   - `global_costmap` 配 `obstacle_layer` 并接激光（如 `/scan`），动态障碍才会进图。
   - 节点订阅的话题是 `/global_costmap/costmap_raw`（`nav2_msgs/Costmap`，0~255 内部编码），**不是** `/global_costmap/costmap`（那是 `OccupancyGrid`，0~100）。若你的 Nav2 带命名空间（如 `/navigation/...`），改参数 `costmap_topic` 对齐。
3. **定位与地图**：`map_server` + `amcl` 正常运行，节点通过 tf 取 `map → odom → base_link` 位姿（`map_frame` 与 `robot_base_frame` 参数需与你的坐标系一致）。

### 启动后验证清单

1. `ros2 node list` 应能看到路网节点（`route_planner.launch.py` 下名为 `route_planner_node`，全栈 launch 下名为 `route_navigator_node`），以及 `planner_server`、`controller_server`、`amcl`、`map_server`；
2. `ros2 action list | grep -E "compute_path_to_pose|follow_path"` 两个 action 均存在；
3. 节点日志出现 `路网导航节点已启动 [ActionServer: /navigate_to_pose]`；
4. `replan_trigger_mode` 含 `monitor` 时，日志应出现 `已收到代价地图: /global_costmap/costmap_raw (...)`；只用 `failure` 模式则不订阅代价地图，无需第 2 条；
5. RViz 用 "2D Goal Pose" 点目标，机器人沿路网行驶。

---

## 下发目标 / 交互

启动后，用 RViz 的 **"2D Goal Pose"** 工具：

1. 点工具栏 **"2D Goal Pose"**；
2. 在地图点目标位置，拖箭头设朝向；
3. 机器人自动规划路网路径并行驶。

- 也可向话题 `/route_planner/goal`（`geometry_msgs/PoseStamped`）发目标，效果等同。
- **不要用 "Navigation2 Goal"**（走 `bt_navigator`，本项目不启动）。
- **不要用 "2D Pose Estimate"** 设目标（那是给 amcl 初始定位用的）。

**两种模式表现：**

- 纯路网（`allow_offroad:=false`）：点路网外的目标会被拒绝；路网上严格沿路网走。
- 三段式（`allow_offroad:=true`）：点路网外目标会先"上路"→路网→"下路"；目标在路网上时，到点后自动转到你设的朝向。

---

## 障碍动态重规划（简述）

路网段行驶中前方遇动态障碍（行人、临时堆放物等），节点自动重规划绕行，无需人工干预。

- **两种触发路径**，由 `replan_trigger_mode` 选择：
  - `"both"`（默认）：代价地图在线检测 + 控制器中止兜底，都触发；
  - `"monitor"`：仅代价地图在线检测；
  - `"failure"`：仅控制器中止兜底（**不订阅代价地图**，也就无需配置 `obstacle_layer`/`always_send_full_costmap`）。
- 重规划以最近路网节点为锚点，临时摘掉堵塞边逼出替代路线，规划后恢复该边；带防无限循环与最大次数限制。

---

## 移植步骤（适配新机器人 / 平台）

本包与具体机器人解耦，移植只需替换"地图 + 路网 + Nav2 参数 + 少量坐标/话题参数"，核心逻辑无需改动。

1. **编译**：`colcon build --packages-select route_planner`，确认 `route_navigator_node` 可启动。
2. **准备地图**：SLAM 或人工绘制的 `*.pgm` + `*.yaml`（单位米，坐标系 `map`），放 `maps/` 或用 launch 的 `map:=` 指定。
3. **绘制路网**：用 `route_editor.py` 在地图底图上画节点/边，导出 `*.geojson` 到 `routes/`；或手写（格式见下）。
4. **配置 Nav2**：用本包 `config/nav2_params_sim.yaml` 作模板，确认 `controller_server` 含 `FollowPath`+`FollowPathRPP`，并满足上文"Nav2 侧条件"。
5. **坐标系/话题参数**：在 launch 或 `config/route_planner.yaml` 中覆盖下表项。
6. **启动验证**：按"启动后验证清单"逐项确认。

| 参数 | 默认值 | 何时需改 |
|---|---|---|
| `map_frame` | `map` | 地图坐标系改名时 |
| `robot_base_frame` | `base_link` | 实车用 `base_footprint` 等时 |
| `costmap_topic` | `/global_costmap/costmap_raw` | Nav2 带命名空间时 |
| `network_controller_id` | `FollowPathRPP` | 你的 RPP 插件命名不同时 |
| `route_file` | 自动查 `routes/` | 指定自定义路网 |

---

## 路网编辑工具

`route_editor.py` 在地图底图上可视化绘制/编辑路网（需 `Pillow`）。

```bash
# 基础启动（无底图）
python3 src/route_planner/route_editor.py

# 加载地图底图（取本包自带或你自己的）
python3 src/route_planner/route_editor.py \
  --map src/route_planner/maps/sim_room.pgm \
  --yaml src/route_planner/maps/sim_room.yaml

# 加载已有路网继续编辑
python3 src/route_planner/route_editor.py \
  --geojson src/route_planner/routes/warehouse_routes.geojson
```

| 操作 | 方式 |
|---|---|
| 添加节点 | 点击空白处 |
| 添加边 | 依次点击两个节点 |
| 移动节点 | 拖拽节点 |
| 删除节点/边 | 选中后按 Delete |
| 编辑节点名 | 双击节点 |
| 保存 / 导出 GeoJSON | `Ctrl+S` 或菜单 File → Export GeoJSON |

---

## 参数速查（`config/route_planner.yaml`）

**基础参数**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `route_file` | `""` | GeoJSON 路网路径，留空自动查找 `routes/` |
| `map_frame` | `map` | 地图坐标系 |
| `robot_base_frame` | `base_link` | 机器人 base 坐标系 |
| `algorithm` | `astar` | 路网规划算法（`astar` / `dijkstra`） |
| `snap_distance` | `3.0` | 最大吸附距离（米），超过认为"不在路网附近" |
| `on_graph_threshold` | `0.5` | 判断"在路网上"的距离阈值（米） |
| `dense_step` | `0.2` | 路网密集化步长（米），越小约束越强 |
| `allow_offroad` | `false` | 是否允许路网外自由导航（三段式） |
| `enabled` | `true` | 是否启用路网约束（`false` 退化为标准 Nav2） |
| `network_controller_id` | `FollowPathRPP` | 路网段控制器插件名 |
| `planner_timeout` | `10.0` | 预留参数，当前版本不生效 |
| `controller_timeout` | `120.0` | 预留参数，当前版本不生效 |

**障碍重规划参数**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `replan_enabled` | `true` | 总开关；`false` 退回纯路网 |
| `replan_trigger_mode` | `both` | 触发模式：`both` / `monitor` / `failure` |
| `cost_threshold` | `60` | 单点代价阈值（0~255），超过记高代价；253≈膨胀边缘，254≈致死 |
| `path_obstacles_threshold` | `2` | 前方 `cost_lookahead` 内高代价点数量达该值即触发 |
| `cost_lookahead` | `1.5` | 仅检查机器人前方该距离（米）内路径点 |
| `monitor_hz` | `2.0` | 障碍监控频率（Hz） |
| `costmap_topic` | `/global_costmap/costmap_raw` | 全局代价地图话题（`nav2_msgs/Costmap`） |
| `replan_reverse_via_freenav` | `true` | 重规划后先用自由导航接驳回锚点；`false` 直接接续当前位姿 |
| `replan_on_failure` | `true` | 控制器中止时也兜底重规划（仅跟踪失败时触发） |
| `replan_max_attempts` | `5` | 单次导航最多重规划次数，防无限重规划 |

---

## 接口清单（集成对接用）

**动作 Action**

- `/navigate_to_pose`（`nav2_msgs/NavigateToPose`）— 本节点作为 **ActionServer** 接收目标；
- `compute_path_to_pose`（`nav2_msgs/ComputePathToPose`）— 本节点作为 **ActionClient** 调用 planner_server；
- `follow_path`（`nav2_msgs/FollowPath`）— 本节点作为 **ActionClient** 调用 controller_server。

**订阅话题**

- `/goal_pose`、`/route_planner/goal`（`geometry_msgs/PoseStamped`）— 目标输入；
- `/global_costmap/costmap_raw`（`nav2_msgs/Costmap`）— 仅 `replan_trigger_mode` 含 `monitor` 时订阅；
- `/tf`、`/tf_static`、`/map` 等由 Nav2 / tf 提供。

**发布话题**

- `/route_path`（`nav_msgs/Path`）— 当前路径，供 RViz 显示；
- `/route_graph_markers`（`visualization_msgs/MarkerArray`）— 路网节点/边可视化。

> 节点自包含：所有逻辑都在 `route_navigator_node` 一个进程内，没有独立的"跟随节点"。
> 集成时只需保证上述 Nav2 服务可用。

---

## GeoJSON 路网格式（简版）

标准 `FeatureCollection`；`Point` = 节点，`LineString` = 边。

```json
{
  "type": "FeatureCollection",
  "properties": { "name": "仓库运输路网", "coordinate_system": "map", "units": "meters" },
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

约定：`coordinates` 为 `[x, y, z]`（米，坐标系与地图一致）；边 `bidirectional: true` 表示双向通行。

---

## 常见问题

**Q: 节点起不来 / 报 ActionServer 不可用**
先确认 Nav2 栈（至少 planner_server、controller_server）已运行且三个 action 可见。本节点不自带 Nav2，必须你先把它跑起来。

**Q: 重规划一直不触发，小车只停障**
1. 看日志是否出现 `已收到代价地图: /global_costmap/costmap_raw`；没有说明话题名/命名空间不符，用 `ros2 topic list | grep costmap_raw` 确认，再改 `costmap_topic`；
2. 确认 global_costmap 配了 `obstacle_layer`（接 `/scan`）且 `always_send_full_costmap: true`；
3. 若只想靠控制器中止兜底，把 `replan_trigger_mode` 设为 `failure`。

**Q: 如何禁用路网约束**
设 `enabled: false`，节点退化为标准 Nav2 自由导航。

**Q: 实车坐标系不同（如用 `base_footprint`）**
启动 launch 时覆盖参数，或改 `config/route_planner.yaml` 的 `robot_base_frame` 与你的坐标系一致。

**Q: 每次到节点都误触发重规划**
正常到达节点不会触发；兜底只在跟踪失败（`result=False`）时触发。确认代码已更新到含 `replan_trigger_mode` 的版本；只想保留监控触发可设 `replan_trigger_mode: monitor`。

---

## 许可证

Apache-2.0
