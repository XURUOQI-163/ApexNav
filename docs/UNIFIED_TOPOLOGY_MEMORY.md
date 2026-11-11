# Unified Topological Memory（UTM）

UTM 保存现有 APEXNAV 的 Skeleton Graph、当前 Frontier 的稀疏语义以及代表性原始 RGB 图像。默认关闭，通过规划器启动参数 `utm:=true` 开启；同时自动开启 Skeleton 观察器。现有 Frontier、Skeleton、语义融合、全局二维占据地图和控制算法均继续工作。没有实现滚动地图或栅格淘汰，也没有接入已暂停的 `--planner TOPOLOGY`。

## 启动

在 `apexnav-ros2-jazzy` 容器中编译（已经完成此次编译；之后自行修改 C++/消息才需要重编）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select plan_env exploration_manager --symlink-install \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

重启规划器，使用新消息/库。终端一：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py \
  planner:=APEXNAV mode:=continuous utm:=true
```

YOLOE、BLIP2 服务仍按之前的方式启动。仿真终端可沿用原命令，例如：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
export ROS_DOMAIN_ID=31
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
python habitat_evaluation.py --mode continuous --planner APEXNAV --semantic-model blip2 \
  --target "chair" --dataset hm3dv2 test_epi_num=800 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets \
  habitat.simulator.habitat_sim_v0.allow_sliding=true
```

离散模式时同时将两处 `mode` 改为 `discrete`。不需要额外下载资源或安装视觉模型。独立记录进程使用容器已有 `/usr/bin/python3`、ROS2、cv_bridge、OpenCV；可以通过 `utm_python:=...` 修改解释器。

需要改变位置时，增加 `utm_directory:=/absolute/path`。默认位置从当前工程源码定位，不依赖运行命令的工作目录。启动器要求 `--symlink-install` 的源码工程可访问。

## 保存位置与生命周期

首次收到有效仿真上下文及新鲜几何快照后生成：

```text
ApexNav-ROS2/data/topology/
├── manifest.json
├── persistent/
│   └── skeleton_graph.json
├── dynamic/
│   └── frontier_state.json
├── observations/
│   ├── junction/J_<id>/O_<unique_id>.jpg
│   └── frontier/F_<session>_<source_id>/O_<unique_id>.jpg
├── snapshots/
│   └── unified_graph_latest.json
└── archives/<map_key>-<unique_suffix>/ ...
```

JSON 有上一版 `.json.bak` 备份；`.writer.lock` 防止两个记录器同时写一个目录。图像写入使用唯一 ID。结束/中断仿真不清空磁盘。

- **仅重启记录器，仿真仍在运行**：加载同一仿真、同一地图实例的 ID、图像、语义与关联，继续更新。
- **重新启动仿真，同一场景且同一起始位姿**：加载长期节点/Edge，暂标记 `UNCONFIRMED`；收到有效的完整骨架快照后按类型和位置匹配，匹配成功保留 ID/图片，未匹配旧记录直接释放。上一轮 Frontier 不当作当前有效前沿，重新根据当前地图构建；旧 Frontier 图片在新快照保存成功后清理。
- **场景或起始位姿改变**：把上一份完整目录内容移到 `archives/`，当前层重新开始。没有跨坐标系合并。
- **本轮全局骨架重建中节点/边消失**：直接移出 UTM 内存和当前磁盘快照，包括关联边及源 ID 索引；不再积累 `RETIRED` 历史。节点仍在当前全局骨架中时，即使离开相机视野也继续保留。DORMANT 前沿仍属于有效的动态前沿，保留节点及图片。

原因：Habitat 的 episodic GPS/Compass 以每个 episode 的起始位姿为基准。即使话题都叫 `world`，换起点后也不是同一坐标。`manifest.json` 保存 `scene_id`、`start_position`、`start_rotation`、坐标转换约定、`frame_id` 和 `map_key`。仿真/地图实例 ID 还用于阻止旧 ROS 消息和重复 source ID 污染新数据。

**恢复初始化的是 UTM 查询层，不向正在导航的 OccupancyGrid、FrontierManager 或语义融合器注入旧状态。** 仅有几何骨架不能重建安全的全局占据地图。本阶段保留用户指定的原导航行为。

### Junction 实际访问状态（2026-11-11）

`status=ACTIVE/UNCONFIRMED` 仍是原有快照生命周期字段，不代表机器人是否访问。仅 Junction 新增 `visit_state=CANDIDATE/CONFIRMED`、`visit_radius`、`confirmed_at_ns` 和 `visit_position`。Endpoint / Loop Anchor 的 JSON 不保存这些字段，ROS 共用消息中使用 `NOT_APPLICABLE`。

新节点为 CANDIDATE。SkeletonGraphManager 复用 `/odom_world`（仿真 remap 到 `/habitat/odom`），依据机器人底盘的世界 XY 坐标，在 0.5m 半径内确认。忽略错误坐标系、过期、乱序和早于地图初始化的里程计。相机扫描只生成候选节点，不能确认。确认后离开不会取消；同 ID 重建继承访问证据，异步旧快照不能覆盖更新的确认。若节点移动到历史访问位置的半径之外，则重新视为候选，不能把旧访问套用到未访问的位置。当前图的节点淘汰机制保持不变，不新增长期保留失效节点的第二张图。

RViz 候选 Junction 为浅绿色，已确认 Junction 为深绿色；Endpoint 仍为红色。候选/确认均可供原规划器选择路径，访问状态不改变 Frontier 评分或导航控制。

```json
{"node_type":"JUNCTION", "status":"ACTIVE", "visit_state":"CONFIRMED",
 "visit_radius":0.5, "confirmed_at_ns":1794405000000000000,
 "visit_position":[2.1,0.4], "x":2.3, "y":0.5, "observations":[]}
```

参数分别为 `skeleton_graph.yaml` 的 `visit_radius: 0.50`、`visit_odom_timeout: 1.0` 和 `topology_memory.yaml` 的 `junction_keyframe_angle_degrees: 60.0`。图片以拍摄时同步的 odom 判断距离，不用保存时的当前位置替代。只有通过筛选的图片才 JPEG 编码，替换后先原子保存引用，再清理旧文件。

旧 JSON 缺少访问证据时默认 CANDIDATE，不把原来的 ACTIVE 或旧照片当作访问证明；不符合新条件的 Junction 图片引用会在记录器迁移/更新时移除，成功保存后回收孤儿 JPEG。已有 archives 不变。单独恢复记录器可从 JSON 读取访问状态；新地图/规划器的实时状态仍由新图和实际里程计决定，不向规划器倒灌历史图。

本次新增 ROS 字段，需重启规划器和 UTM 记录器（当前 `simulation.launch.py ... utm:=true` 一起启动）。编译使用 `colcon build --packages-select plan_env path_searching trajectory_manager exploration_manager topology_exploration --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON`。仿真和视觉模型命令不变，注意只运行一套规划器。

## 数据含义

`persistent/skeleton_graph.json` 的简化示例（实际还包括像素、锚点等字段）：

```json
{
  "schema_version": 1,
  "frame_id": "world",
  "map_key": "...",
  "nodes": [
    {"node_id": "J_a1", "source_id": 14, "node_type": "JUNCTION",
     "layer": "PERSISTENT", "status": "ACTIVE", "x": 1.5, "y": 2.0,
     "clearance": 0.4, "observations": []},
    {"node_id": "J_b2", "source_id": 25, "node_type": "JUNCTION",
     "layer": "PERSISTENT", "status": "ACTIVE", "x": 3.0, "y": 2.5,
     "clearance": 0.5, "observations": []}
  ],
  "edges": [
    {"edge_id": "E_e1", "edge_type": "SKELETON", "status": "ACTIVE",
     "start_node_id": "J_a1", "end_node_id": "J_b2",
     "path": [[1.5, 2.0], [2.0, 2.5], [3.0, 2.5]],
     "length": 1.7071067812, "min_clearance": 0.4, "mean_clearance": null,
     "traversability": "SKELETON_PATH_ONLY_REQUIRES_LOCAL_VALIDATION"}
  ]
}
```

关键区别：

- `path` 是 SkeletonEdge 的**完整实际路径**。删除距离合并后，新数据的 `display_path` 与 `path` 一致；旧记忆中的 `display_path` 可能包含抽象中心补线，仍须以 `path` 为实际几何依据。
- 已停止产生距离合并的 Junction，但保留旧记忆加载机制。旧记录可能存在中心与实际路径端点不同的情况；UTM 不宣称历史图已适合直接执行运动，后续规划仍须验证当前局部通行性。
- 保留现有图中可能出现的 `LOOP_ANCHOR`（以及显式开启保留死端时的 `ENDPOINT`），避免破坏环路和边引用，不强行把所有节点改名成 Junction。
- 现有消息没有平均 Clearance，`mean_clearance` 为 `null`，没有拿最小值冒充平均值。

`dynamic/frontier_state.json` 中每个当前 Frontier 示例：

```json
{
  "node_id": "F_<session>_7",
  "source_id": 7,
  "node_type": "FRONTIER",
  "layer": "TRANSIENT",
  "status": "ACTIVE",
  "x": 2.5,
  "y": 1.5,
  "parent_junction_id": "J_a1",
  "approach_pose": {
    "x": 1.5, "y": 1.5, "yaw": 0.0,
    "source": "known_free_attachment_support_not_controller_goal"
  },
  "goal_id": "...",
  "semantic": {
    "goal_text": "chair",
    "model_name": "blip2",
    "model_version": null,
    "raw_similarity": 0.23,
    "raw_frame_stamp_ns": 1793710000000000000,
    "computed_at": 1793710000300000000,
    "fused_score": 0.21,
    "confidence": 0.5,
    "normalized_score": null,
    "aggregation": "max_of_fused_frontier_cells",
    "scored_cells": 1
  }
}
```

- `fused_score`：已有算法计算的前沿格子融合值的最大值。UTM 不再持久化 `cells` 逐格列表；原有 FrontierManager 的逐格融合算法保持不变。
- 节点 `raw_similarity`：取得最大融合分数的那个格子所对应的最近原始图像分数，不是所有图片的平均值，也不等于融合值。
- `computed_at`：收到该帧语义处理元数据时的实际计算完成时间（纳秒）。记录器启动较晚、证据缓存缺失时为 `null`，不拿接收时间伪造。原始帧时间单独保存。
- 当前算法没有归一化语义概率，因此 `normalized_score=null`；不知道服务权重版本时 `model_version=null`，可在配置中填写实际权重标识。
- `approach_pose` 是关联路径的已知自由支撑点及朝向前沿的方向，不修改或冒充运动控制器选择的目标。没有有效连接时为 `null`。
- `DORMANT` 前沿仍作为当前几何记录保存，但 `get_active_frontiers()` 只返回 `ACTIVE`。

`attachments` 独立保存 `edge_type=FRONTIER_ATTACHMENT`、Junction/Frontier ID。2026-11-05 起，`path_role=KNOWN_FREE_APPROACH` 的 `path`、`raster_path` 和 `length` 只描述 Junction 到自由接近点的安全路径；允许 Junction 已在接近点时保存单点、零长度路径。`association_path=[approach, centroid]` 和 `association_length` 单独保存到前沿球心的短关联段，明确 `association_traversable=false`；`terminal_may_be_frontier_unknown=false` 表示安全路径不包含 Unknown。关联仍不是 Junction 间的 Skeleton Edge。旧磁盘记录保留原样，未包含新标记的历史附件不能直接按新路径语义解释；记录器遇到旧发布者的消息时使用其自由栅格路径，避免把旧显示路径中的 Unknown 当作自由空间。

## 图像

Observation 示例：

```json
{
  "observation_id": "O_<unique_id>",
  "image_path": "observations/frontier/F_<session>_7/O_<unique_id>.jpg",
  "stamp_ns": 1793710000000000000,
  "frame_id": "world",
  "camera_pose": [0.5, 1.5, 0.88, 0.0, 0.7071, 0.0, 0.7071],
  "robot_pose": [0.5, 1.5, 0.0, 0.0, 0.0, 0.0, 1.0],
  "camera_yaw": 0.0,
  "bearing_to_node": 0.0,
  "evidence": {
    "goal_text": "chair", "model_name": "blip2", "model_version": null,
    "raw_similarity": 0.23, "computed_at": 1793710000300000000,
    "projected_cells": [{"token": 12, "weight": 0.5}]
  }
}
```

Pose 数组顺序是 `[x,y,z,qx,qy,qz,qw]`，方向为弧度，图片是未画框的 RGB 原图转 JPEG。图片路径相对 `data/topology/`。

- 使用已有 RGB + FrontierScan + Odometry 的精确时间同步、原始射线自由格支持与 `project_cells()`，不根据今天的全局地图反推旧图可见性。
- Junction 只有在原始相机帧确实扫描到其位置且在前向视野内时选图；没有观测到就保留空列表。没有机器人到达要求，也不生成虚假图片。
- `save_frontier_images=false` 默认关闭 Frontier JPEG 和观测引用保存。打开后保留原有流程：Frontier 优先选择真正参与语义计算的原始帧；慢 BLIP 返回时，使用其持有的原始图片与原始位姿。若先保存了同帧的无分数图片，后续只补入语义证据。同一帧分批支持新格子时合并证据。
- 无分数几何图像会明确标记 `evidence=null`，之后可用同一视角的真实评分帧升级。当前节点的观测引用更新为新图，旧 JPEG 在成功保存后清理。
- Junction 最多 4 张：节点须为 `visit_state=CONFIRMED`，且拍摄时机器人底盘位置距节点不超过 `visit_radius`（默认 0.5m）。记录到达路口时各朝向的场景，不要求画面中心正对路口中心。新朝向与所有已有朝向的差均严格大于 60° 才新增；处于已有视角范围内且机器人距节点更近时替换该图，不增加数量。恰好 60° 归同视角；若同时落入两个已保存视角范围，跳过以免替换产生重复。满 4 张仍可择近替换，但不新增。
- Dead-end（ENDPOINT）不设访问确认状态，最多 1 张，继续要求原始深度扫描可见并在相机视野内。优先使用相机朝向更正对节点（朝向与节点方位的夹角更小）的图；夹角相同则优先更近的图。可远处拍摄，更新采用替换。
- LOOP_ANCHOR 和显式启用 JPEG 的 Frontier 保留原角度/位移规则（45° 或 0.75m）；Frontier JPEG 默认仍关闭。`junction/` 目录覆盖全部骨架节点，包括 ENDPOINT。没有图像内容相似度、清晰度模型或跨节点去重。**只编码和保存被选中的图片，不额外调用视觉模型。**
- Junction 不存目标语义属性。Frontier 图像证据和当前分数带目标文本/模型信息。

前沿只保存当前状态，不收集或写入事件历史。旧版本留下的事件文件不参与恢复，也不再追加或修复；加载旧 manifest 时清除已停用的历史路径引用。

## 更新、恢复与查询

加载旧数据时跳过 RETIRED 等失效记录，第一次成功写盘后清理当前 `observations/` 中无节点引用的 UTM JPEG；节点移除或代表图替换也触发清理。先原子更新 JSON，再删旧图；写盘失败则保留图片等待重试。仅扫描当前层，不处理 `archives/`。`.json.bak` 仍保留一代备份，恢复时缺失的图片引用会被剔除。

独立进程中的 ROS 回调只保存最新消息/有限帧引用。默认每秒一个后台工作周期，完成结构转换、选图、JPEG 和 JSON 写盘。JSON 内容未变时不改文件；Skeleton 不因相机帧或发布序号变化而重写。

临时文件先显式设置权限 `0644`，再 `fsync` 和原子替换，使容器 root 写出的图片/JSON 可由宿主机用户读取；先落图片再写引用；有上一版 JSON 备份，载入检查 schema、地图身份、ID 唯一性、边端点、路径数值、图像相对路径及文件存在性。损坏 JSON 尝试备份；坏引用报告后移除；不能确认地图身份时停止写入并报告，保留文件供检查。

这是**各 JSON 文件的原子更新**，不是多个文件的数据库事务。异常退出可能丢失最后一次尚未提交的更新；载入时检查并清除跨版本的悬空关联。统一快照只由两层生成，不用于恢复导航数据。

运行中查看状态：

```bash
ros2 topic echo /topology_memory/status --once
```

停止后，在项目根目录执行（离线查询不需要 ROS 或视觉环境）：

```bash
python -m topology_memory
python -m topology_memory --node J_实际节点ID
```

恢复验证方法：先停止记录器，记录上述查询输出，再重复加载，检查节点数、路径、图片引用不变；同起点重新运行后观察 Junction 复用与 `UNCONFIRMED -> ACTIVE`，不同起点检查 `archives/`。只重建调试快照可使用 `python -m topology_memory --export`，要求记录器未占用写锁。

查询 API：

```python
from topology_memory import UnifiedTopologyMemory

memory = UnifiedTopologyMemory('data/topology')
issues = memory.load_memory()
graph = memory.get_topology_graph()  # 默认仅 ACTIVE；可 include_inactive=True
frontiers = memory.get_active_frontiers()
if frontiers:
    parent = frontiers[0]['parent_junction_id']
    related = memory.get_frontiers_by_junction(parent)
    images = memory.get_node_observations(frontiers[0]['node_id'])
# save_skeleton_graph / save_frontier_state / save_observation 更新管理对象；
# 生产中只由独立后台 worker 调用，并在批末 flush()，不要在 ROS 回调写盘。
```

## 文件与调用关系

```text
SDFMap2D / ValueMap / FrontierManager（原实现）
  ├─ SkeletonGraphManager -> /skeleton_graph/graph（原接口）
  └─ MapROS::publishFrontierMemory -> /topology_memory/frontier_state（只读）
Habitat simulator -> /topology_memory/context（场景、起点、任务）
RGB + FrontierScan + Odometry -> 精确同步的 20 帧窗口
FrontierSemanticBridge -> /topology_memory/scored_observation（原始计算帧）
                     ↓
UnifiedTopologyMemoryNode（采集）
  -> MemoryRecorder（后台适配/选图）
  -> UnifiedTopologyMemory（分层存储、加载、查询）
```

新增核心文件：`topology_memory/{store,recorder,ros_node,__main__,__init__}.py`。

新增 C++ 只读导出：`src/planner/plan_env/src/map_ros_memory.cpp`，新增四个消息 `FrontierMemoryCell/Node/State.msg`、`TopologyObservation.msg`。在 MapROS 的头文件/初始化及 plan_env CMake 中登记，没有修改 Frontier、Skeleton 核心算法。

图像/场景接入：`habitat2ros/topology_context.py`；`habitat_evaluation.py`、`habitat_vel_control.py` 每轮 reset 后发布身份；`habitat2ros/frontier_semantics.py` 增加有订阅者时才启用的原帧输出。

启动/配置：`src/planner/exploration_manager/launch/simulation.launch.py`、`config/topology_memory.yaml`、`package.xml`。C++ 快照默认 1Hz，可用节点参数 `topology_memory.snapshot_hz` 调整；无订阅者时不构造快照。

检查文件：`tests/test_unified_topology_memory.py`、`tests/utm_ros_smoke.py`。小型合成检查写入临时目录，不生成真实场景数据、不运行 Habitat 实验。

RViz 继续使用已有 Skeleton/Frontier 显示，不添加第二套重复 Marker。磁盘内容通过 JSON/图片及以上查询命令检查。

## 此次验证

`plan_env`、`exploration_manager` 在 Jazzy 容器中编译通过，保留了工程原有 CMake/PCL 和弃用接口警告。8 项存储检查、4 项合成 ROS/图像检查通过；另通过原有 6 项图像窗口、8 项仿真模式/速度、1 项稀疏语义融合、2 项 C++ 地图/目标融合检查，共 29 项。

合成样例记录了 2 个 Junction、1 个 Skeleton Edge、1 个 Frontier 和 1 条 Attachment；检查了 RGB 原图颜色、原始时间戳、延迟语义补全、重复帧证据合并、备份/中断恢复与场景隔离。没有运行真实 Habitat 场景测评，真实节点数和关键帧效果留待仿真中检查。

复查新增测试：

```bash
python -m unittest discover -s tests -p test_unified_topology_memory.py -v
# 容器内，先 source ROS 与 install/setup.bash：
ROS_DOMAIN_ID=94 /usr/bin/python3 -m unittest discover -s tests -p utm_ros_smoke.py -v
```

## 失效记录轻量化检查（2026-11-11）

新增检查覆盖完整快照移除节点/边和索引、DORMANT 保留、图片替换和孤儿图清理、写盘失败时延迟删除、旧轮次待确认记录淘汰、archives 保持，以及关键帧角度/位移 OR 规则。该更新仅修改 Python 存储/记录层；重启 UTM 进程后生效，不需要重编 C++。不会主动中断正在运行的仿真或改写其数据；已有无效数据由新进程收到有效快照后清理。

## 图片开关与分类型上限

在 `src/planner/exploration_manager/config/topology_memory.yaml` 中配置：

```yaml
save_frontier_images: false
max_frontier_observations_per_node: 8  # 仅开启 Frontier JPEG 时使用
max_junction_observations_per_node: 4
max_deadend_observations_per_node: 1
```

替代原统一 `max_observations_per_node`。参数在记录器启动时读取；改为 `save_frontier_images: true` 并重启记录器可恢复 Frontier 原始帧选图与保存。关闭时 UTM 的语义缓存只留计算元数据，不长期保留对应 RGB；Junction 选图仍使用有限长度的 RGB 窗口。Frontier JSON 仅含身份、位置、任务语义摘要、状态及拓扑附着数据，不保存图片引用或逐格列表。开启 JPEG 后仅恢复图片观测引用，逐格列表仍不落盘。

旧数据加载时同样应用数量上限：按原观测列表顺序保留前 4/1 张，不新增图像质量排序。超额图片和禁用的 Frontier 图片在新快照成功写入后清理；archives 不变。改动仅涉及 Python 与 YAML，重启 UTM 即可，无需重编 C++。
