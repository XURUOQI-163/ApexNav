# Skeleton Graph：独立的长期几何骨架

这是 APEXNAV 全局二维几何地图的只读观察模块，使用 `skeleton:=true` 开启。当前默认移除红色 Deadend，合并距离小于 2m 的绿色 Junction，并将现有动态 Frontier 关联至绿色节点；显示连线接到原前沿节点中心。它不使用已暂停的 `--planner TOPOLOGY` / 深度描述子图，不输出运动指令，不改变 Frontier 生命周期、语义投影、目标选择或 MPC。本阶段不保存语义、图片和磁盘快照。

“长期”指以当前 episode 累积的全局已知自由空间为基础，不随相机视野和动态 Frontier 删除而遗忘；不是跨仿真保存。退出仿真保留图，下一轮原 `/exploration/reset` 重建地图时统一清空。全量重建可能因真实几何变化、分支剪枝而移除已失效节点。

## 地图来源与接入

当前 APEXNAV 的基础地图是 `SDFMap2D` 内部 log-odds 占据数组，已有 `/grid_map/*` 发布主要为 PointCloud2；本模块不订阅这些点云，也不新增全局 OccupancyGrid 缓存。

```text
原深度/位姿同步 -> 原点云/射线更新 -> SDFMap2D（原有全局几何地图）
                                      |
                     MapROS owns SkeletonGraphManager
                                      |
               定时只读 getOccupancy / getInflateOccupancy
                                      |
            临时三态+膨胀标志图 -> 小块指纹变化检查
                                      |
               单个后台任务：细化 -> 剪枝 -> 图压缩/ID匹配
                                      |
                 去除 Endpoint -> 关联现有 Frontier（安全折线）
                                      |
              /skeleton_graph/graph + /skeleton_graph/markers
```

- `SDFMap2D::GRID_STATE` 为 UNKNOWN=0、FREE=1、OCCUPIED=2，越界 getter 返回 -1；不能直接按 ROS OccupancyGrid 的 -1/0/100 解读。
- 二值可通行图只包含 **FREE 且 getInflateOccupancy()==0** 的格子。Occupied、Unknown、膨胀区域均为 0。没有写入原占据缓冲区。
- 临时图 `row=y、column=x`，与原内部数组 `x*height+y` 布局不同，逐格通过 getter 读取，避免转置。
- 地图 origin、resolution、尺寸由 `getRegion()/getResolution()` 获得，frame_id 使用 `map_ros/frame_id`；世界坐标为 `origin + (pixel+0.5)*resolution`。原 SDFMap2D 轴对齐，yaw=0。纯几何模块也支持带 yaw 的原点，已测试。
- 定时器使用地图更新所在默认互斥回调组，采样期间不会与地图更新交叉。工作线程只持有临时图和旧骨架图副本，不访问 SDFMap2D 或 ROS。最多一个任务，没有无限积压队列。
- 保存的只有当前骨架图和每块的 64 位指纹。临时像素图、距离场、细化图在任务结束后释放，不保留第二份全局占据地图。

## 提取与压缩

1. 在已知自由空间的包围框及必要边界上下文内进行全量细化，图像外补 0。
2. 当前 Jazzy 容器已有 OpenCV 4.6 `ximgproc::thinning(THINNING_ZHANGSUEN)`，直接复用。构建环境没有 ximgproc 时，使用模块内轻量 Zhang-Suen 后备实现，无需安装额外依赖。
3. 八邻域中若一条对角连接已有正交骨架路径替代，去掉该对角捷径，避免 L/T 处的三角重复连接。真正只有对角邻接的路径仍然保留。
4. 度为 1 的像素是 Endpoint，度为 2 的 Connector 只进入边路径，度大于等于 3 的相邻像素聚成 Junction。代表点选择簇内实际骨架像素；边通过簇内骨架连接至代表点，不画穿越障碍的质心直线。
5. 沿 Connector 追踪并压缩成 Edge；保留全部像素和世界坐标路径，长度为相邻路径段的欧氏距离和。
6. 没有 Endpoint/Junction 的闭环设一个 `LOOP_ANCHOR` 和一条首尾闭合的 self-edge。允许两节点间多条不同路径，不能简单按端点 ID 去重。
7. 按实际米制长度剪除短的 Endpoint–Junction 叶分支，重复压缩直到稳定。此几何剪枝默认保护 Unknown 附近的 Endpoint；不根据通道宽度删除。单独的孤立像素被丢弃。
8. 消除填满 2×2 骨架像素面造成的假环，包括多个相邻填满面的组合。用无向像素边的 XOR 基判别冗余路径，不删除连接外部通道的像素；真实 Unknown/障碍孔洞周围的环保留。
9. `remove_deadends=true` 时再统一过滤邻居为 1 的正式 Endpoint 及其关联边，包括 Unknown 附近的端点和独立 Endpoint–Endpoint 走廊。这是一次性端点过滤，不递归删除绿色 Junction：即使 Junction 暂时只剩 0/1 个骨架邻居，也保留供动态 Frontier 关联。Loop Anchor 不属于 Deadend。设置 false 可查看原始端点图。
10. `junction_merge_distance=2.0` 时反复合并当前距离最近且小于阈值的两个 Junction 簇；两个点精确取中点，多点取原成员等权平均，避免反复取两点中点导致偏移。阈值使用世界坐标欧氏距离，恰好 2m 不合并，0 关闭。Endpoint/Loop Anchor 不参与。合并后更新节点 ID 对应关系、外部边和邻居；内部生成树边折叠，非树边和原自环保留为图中的环。

合并中心是抽象拓扑位置，可能不在骨架上甚至落在障碍/Unknown 中，按请求不强行吸附回自由格。`pixel_x/y` 表示该中心所在栅格；`anchor_pixel_x/y` 保留合并前的真实骨架位置。`clearance`、`near_unknown_boundary` 查询中心所在格，阻塞格 clearance=0。节点合并不证明成员之间可直接通行。本模块只用于表示和显示，原 ApexNav 控制不使用合并中心。

Clearance 是**膨胀后可通行区域到非通行边界的保守余量**，包含 Unknown 和地图外部。由临时欧氏距离场减去半个格子的对角长度后换算成米，不等于原始障碍点中心距离，也不替代控制器原有碰撞判断。节点取代表像素 Clearance，边取完整路径的最小值。

## 更新和节点身份

默认最多每秒检查一次，不在深度回调里细化。状态没有变化时不重建，log-odds 数值小幅变化但三态/膨胀分类相同也不重建。指纹比较基准为**上次成功构图的输入**，小变化会累计，不会被每次轮询覆盖掉。

节点按相同类型、世界坐标距离由近到远一一匹配，匹配距离可配置；未匹配的新节点分配递增 ID。全量细化的骨架可能随自由区域扩张而移动，因此 ID 为尽力保留，不是所有形变下都保证不变。合并后的 Junction 使用最终中心参与类型/距离 ID 匹配；首次合并或合并分组变化时可能分配新 ID。当前有效图中的节点 `active=true`，不保留无效历史节点表。Unknown 边界端点使用 `near_unknown_boundary=true` 标记暂时性。

## 动态 Frontier 连接（2026-11-03 更新）

直接读取同一 `SDFMap2D` 所属 `FrontierMap2D::activeFrontiers()/dormantFrontiers()`，沿用原 Frontier ID、几何质心、成员栅格及 active/dormant 状态，不重新提取或评分，不复制图片或语义数据。

前沿连接从 **当前 Junction 中心** 所在的自由格展开，不再使用合并前的锚点作为起点。先用八邻域多源 Dijkstra 求栅格最短路径，代价包含中心到栅格中心及终点段；对角移动要求两个正交侧格也可通行。之后逐段做栅格射线检查，删除可以直接跨越的中间折点；无遮挡时从当前绿点到原 Frontier 中心直接连成一段。若其他 Junction 存在更短的直视连线，也会选用该连线。绕障路径是栅格最短路加可视简化，不宣称连续平面中严格全局最短。

原 Frontier 的成员格子是 Unknown，非凸边界的质心也可能落在 Unknown/障碍中。`display_path` 从当前 Junction 中心重新计算至 `frontier_centroid`，不再拼接“新中心→旧节点→远处边界→前沿中心”。只允许最后一个格子为属于此 Frontier 的 Unknown，其余全部须 FREE 且未膨胀。`path/pixel_x/pixel_y` 保存最终简化路线经过的自由格作为几何依据，`approach` 是最后一个自由格的中心。RViz 使用 `display_path`。最后一个 Marker 点的高度为 0.36m，与原 Frontier 球体/方块中心相同，`F<id>` 标签也位于原前沿中心旁；不再绘制额外的接入小球。

Frontier 的 `length` 为最终 `display_path` 的完整长度，包含端点段。节点中心在障碍/膨胀/Unknown 内时不作为可达起点；前沿中心落在障碍、地图外或不属于当前 Frontier 的 Unknown 中时发布 `FRONTIER_CENTER_BLOCKED`，不再补未经检查的长连线。每次发布前也对当前地图重新验证简化后的整条线段，不能仅检查简化前的栅格路径。蓝色骨架 Edge 仍保留原始骨架几何及抽象中心显示连接；本次仅修正青色 Frontier 关联，不改变导航控制。

- Frontier 增删、成员变化、active/dormant 变化都会刷新连接，即使占据地图没有变化。
- 任何地图分类块变化也会刷新连接，即使不足以触发骨架全量重建；打开一格门或捷径后可以重新连通、重新选择最近 Junction。连接刷新不覆盖骨架变化累计基准。
- 异步结果提交时再次核对 Frontier 身份与成员、当前地图的路径可通行性；过期 Frontier 记录直接丢弃，失效路径标记 `MAP_CHANGED` 并重新计算。
- 没有绿色 Junction、没有安全接入点、不可达时，分别发布 `NO_JUNCTION`、`NO_FREE_APPROACH`、`UNREACHABLE`，`connected=false`，不生成虚假的连线。纯直线走廊过滤 Endpoint 后可能没有 Junction，这是本版“只连绿色路口”的明确边界；原 ApexNav 探索照常运行。
- 这些关联只是图数据和可视化，不修改全局目标选择、局部规划或速度输出。骨架节点的 `neighbor_count` 仍只统计骨架邻居，动态关联单独存于 `frontier_links`。

## 数据接口

- `/skeleton_graph/graph`：`plan_env/msg/SkeletonGraph`，Reliable + Transient Local，深度 1。包含地图元数据、revision、采样时间、统计、节点、完整边和当前 `frontier_links`。
- `/skeleton_graph/markers`：`visualization_msgs/msg/MarkerArray`，同样保留最新一份供后打开的 RViz 订阅。

节点字段：`node_id`、`node_type`（0 ENDPOINT / 1 JUNCTION / 2 LOOP_ANCHOR）、`pixel_x/y`、世界坐标 `position`、合并前 `anchor_pixel_x/y`、`neighbor_node_ids`、`neighbor_count`、`clearance`、`near_unknown_boundary`、`active`。

边字段：`edge_id`、`start_node_id`、`end_node_id`、原始完整 `pixel_x/y` 和世界坐标 `path`、中心到中心的 `display_path`、几何 `length`、`min_clearance`。邻居数是去重后的邻居 ID 数；闭环自身算一个邻居，不能将该数误作包含重边的图论度数。

新增 `SkeletonFrontierLink` 字段：`frontier_id`、`active`、`connected`、`junction_node_id`、原 `frontier_centroid`、安全 `approach`、完整 `pixel_x/y` 和世界坐标 `path`、中心到中心的 `display_path`、该完整连线的米制 `length`、`reason`。只有 `connected=true` 时路径与接入位置有效。图消息增加 `removed_deadends`、`merged_junctions`、`geometry_rebuilt`、`frontier_link_ms` 统计。

Marker 每次先 DELETEALL，再发布完整当前图：边蓝色，Endpoint 红色，Junction 绿色，Loop Anchor 紫色，默认标节点 ID；默认不再有红色 Endpoint。前沿连接为直达原 Frontier 球体/方块中心的青色折线，标签 `F<id>`。Marker 仅提升 z 避免被栅格覆盖，图消息的几何 z=0。

## 参数

文件：`src/planner/exploration_manager/config/skeleton_graph.yaml`。阈值和显示尺度均集中在这里。修改后重启规划器。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `update_hz` | 1.0 | 轮询和提交构图的最大频率；后台忙时跳过，不堆积 |
| `change_tile_size` | 32 | 指纹块边长，像素；0.05m 分辨率下为 1.6m |
| `min_changed_tiles` | 1 | 至少多少块分类变化才重建，可调大减少微小更新 |
| `min_changed_ratio` | 0.0 | 另一触发条件，变化块/全部块；0 关闭，与块数条件为 OR |
| `min_branch_length` | 0.30m | 叶分支长度阈值；0 关闭剪枝 |
| `unknown_boundary_distance` | 0.50m | 到 Unknown 像素中心的距离阈值 |
| `protect_unknown_endpoints` | true | 几何短分支剪枝保护；不阻止后续统一移除 Deadend |
| `remove_deadends` | true | 移除正式单邻居 Endpoint 与关联边，保留绿色 Junction |
| `connect_frontiers` | true | 当前 active/dormant Frontier 连接最近可达 Junction |
| `junction_merge_distance` | 2.0m | 小于阈值的 Junction 按当前最近簇合并，0 关闭 |
| `frontier_visualization_z` | 0.36m | 前沿连线终点高度，与现有前沿节点一致 |
| `node_match_distance` | 0.40m | 同类节点 ID 匹配最大距离 |
| `visualization_z` | 0.12m | 显示高度 |
| `node_scale / edge_width / label_scale` | 0.16 / 0.035 / 0.14m | 节点、连线、字大小 |
| `show_node_ids` | true | 显示节点编号 |

开启由启动参数 `skeleton:=true` 控制；默认 false，以保持原启动行为。`topology:=none` 关闭旧深度描述子观察层。这两个参数独立。

## 编译与启动

在现有 Jazzy 容器内：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-up-to exploration_manager --symlink-install \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py \
  planner:=APEXNAV mode:=continuous topology:=none skeleton:=true
```

另一个容器终端按原环境启动仿真，明确保留 `--planner APEXNAV`，例如：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
export ROS_DOMAIN_ID=31
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
python habitat_evaluation.py --mode continuous --planner APEXNAV \
  --semantic-model blip2 --target "chair" --dataset hm3dv2 test_epi_num=800 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets \
  habitat.simulator.habitat_sim_v0.allow_sliding=true
```

原 APEXNAV 仿真仍按原需要运行 YOLOE/BLIP2 服务；骨架模块自身不调用这些服务。离散模式也支持 `planner:=APEXNAV mode:=discrete skeleton:=true`，仿真端模式需一致。

RViz 终端加载相同 ROS 和 install 环境并设置相同 ROS_DOMAIN_ID 后：

```bash
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
  ros2 launch exploration_manager rviz.launch.py
```

检查 Fixed Frame=`world`，勾选新增 **Skeleton Graph** 和原 `Mapping_2D`。Frontier 的显示可独立开关；关闭旧 `Sparse topology` 和 `Topology ...` 显示项。如果使用自存 RViz 配置，Add → MarkerArray → `/skeleton_graph/markers`，Durability 选 Transient Local。新图应覆盖已知自由区，并沿实际通道弯折，不是端点间直线。

```bash
ros2 topic echo /skeleton_graph/graph --once --field nodes
ros2 topic echo /skeleton_graph/graph --once --field edges
ros2 topic echo /skeleton_graph/graph --once --field frontier_links
ros2 param get /exploration_node skeleton_graph.enabled
```

## 参考范围

借鉴 [SSMG-Nav](https://arxiv.org/abs/2603.01813) 的骨架空间记忆思想，以及 [STGPlanner](https://arxiv.org/abs/2412.13664) 的几何骨架图表示。STGPlanner 论文使用增量波前骨架提取；本实现按本阶段需求使用全量 Morphological Thinning，不声称复现其增量算法、语义模型或探索策略。

## 本次修改文件

| 文件（相对工程根目录） | 用途 |
|---|---|
| `src/planner/plan_env/include/plan_env/skeleton_graph.h` | 独立几何结构、配置、变化指纹接口 |
| `src/planner/plan_env/src/skeleton_graph.cpp` | 细化、邻接去重、节点聚类、完整边追踪、剪枝、ID 匹配 |
| `src/planner/plan_env/include/plan_env/skeleton_graph_manager.h` | 只读 SDFMap2D 管理器接口 |
| `src/planner/plan_env/src/skeleton_graph_manager.cpp` | 低频采样、后台任务、生命周期、ROS 图和 Marker 发布 |
| `src/planner/plan_env/msg/SkeletonNode.msg`、`SkeletonEdge.msg`、`SkeletonGraph.msg` | 可直接检查的完整图消息 |
| `src/planner/plan_env/include/plan_env/map_ros.h`、`src/map_ros.cpp` | 持有并按开关创建管理器，原深度/前沿回调不插入骨架计算 |
| `src/planner/plan_env/include/plan_env/sdf_map2d.h` | 仅修正 getter 的三态枚举说明注释，未改其行为 |
| `src/planner/plan_env/include/plan_env/skeleton_frontier_links.h`、`src/skeleton_frontier_links.cpp` | 安全 Frontier 接入、最短可通行连接、生命周期签名 |
| `src/planner/plan_env/msg/SkeletonFrontierLink.msg` | 动态连接完整数据 |
| `src/planner/plan_env/tests/skeleton_frontier_links_test.cpp` | 连接、隔墙绕行、过期前沿、单格开门回归 |
| `src/planner/plan_env/CMakeLists.txt` | 编译模块、生成消息、注册测试及可选 ximgproc 检测 |
| `src/planner/exploration_manager/config/skeleton_graph.yaml` | 集中配置 |
| `src/planner/exploration_manager/launch/simulation.launch.py` | `skeleton` 开关和模式检查 |
| `src/planner/exploration_manager/launch/algorithm.launch.py`、`algorithm_traj.launch.py` | 加载配置并将开关传到原探索节点 |
| `src/planner/exploration_manager/config/ApexNav.rviz`、`ApexNav_Traj.rviz` | 独立 Skeleton Graph 显示项 |
| `src/planner/plan_env/tests/skeleton_graph_test.cpp`、`skeleton_manager_test.cpp` | 核心几何、SDFMap2D/ROS 管理器验证 |
| `tests/ros2_skeleton_graph.py` | 实际启动文件、RGB-D 建图、膨胀排除、原前沿共存、重置回归 |
| `docs/skeleton_graph.md`、`docs/superpowers/plans/2026-11-03-skeleton-graph.md` | 用法、实现边界、计划与验证记录 |

## 初版验证记录（未开启统一 Deadend 过滤）

合成地图结果（OpenCV 和内置 Zhang-Suen 两个实现均覆盖）：

| 地图 | 检查结果 |
|---|---|
| 直线厚走廊 | 2 Endpoint，1 Edge |
| T 形走廊 | 3 Endpoint，1 Junction，3 Edge |
| 十字走廊 | 4 Endpoint，1 Junction，4 Edge |
| 多分支走廊 | 4 Endpoint，2 Junction，5 Edge，中间 Connector 已压缩 |
| 0.3m 噪声叶分支、剪枝阈值 0.4m | 剪除后 2 Endpoint、1 Edge；长窄分支保留 |
| 同一短分支面向 Unknown | 标记 near_unknown_boundary，默认不剪除 |
| 闭合矩形环 | 1 Loop Anchor、1 闭合 Edge，完整周长 20m |

额外覆盖：L 拐角不增加假分支、真正对角线保持连通、地图边缘、孤立像素、非零 origin/旋转/分辨率、重复构图和小幅扩展保持 ID、变化阈值累计、厚十字路口、平行通路、膨胀障碍截断、全 Unknown/Occupied、frame 变化不错误匹配。审查发现并修复 Junction 簇内部闭环被吞掉的问题，新增菱形小环及四个外分支的回归：合并后 5 节点、5 边，保留一条闭环边。

真实 SDFMap2D 管理器测试验证读取前后占据和膨胀查询结果完全不变、坐标元数据和完整折线 Marker 正确、地图不变不重建、后台任务期间重建地图不复活旧图、模块无速度发布者。

实际 ROS 启动回归使用 domain 94 和合成归一化深度帧，80m×80m、0.05m 分辨率的原 APEXNAV 地图，生成 N4/E3；路径排除了真实地图膨胀区域，原 Frontier 正常共存，没有旧 `/sparse_topology/markers` 发布者，`/cmd_vel` 仍只有原控制器一个发布者。结束保留图、下一轮 reset 清图、后续场景独立通过。本次采样约 17.8ms，裁剪区域骨架构建约 3.9ms；这不是大场景最坏耗时保证。日志：容器 `/tmp/skeleton-ros-6s1lqa03/planner.log`。

最终编译通过（7 个依赖包，闭环修正后 plan_env 再编译通过）；plan_env CTest 5/5、旧 topology_exploration CTest 7/7、原 Python 50/50 通过。原 APEXNAV 连续/离散模式的延迟语义对齐、结束保留和启动重置回归均通过。YAML、RViz 配置、启动脚本语法和 git diff --check 通过。构建中仍有原 PCL/CMake/GCopter 警告，没有构建失败。

没有进行新的 HM3D 长程探索评测或实车试验。本模块忠实保留已知可通行空间的连通结构；家具造成的环路和长分支仍可能存在，本阶段不做房间分割或语义简化。

复现测试：

```bash
# 已 source ROS 和 install
ctest --test-dir build/plan_env --output-on-failure
ctest --test-dir build/topology_exploration --output-on-failure
source /opt/apexnav_ros2_py312/bin/activate
PYTHONPATH=.:$PYTHONPATH python tests/ros2_skeleton_graph.py
PYTHONPATH=.:$PYTHONPATH python -m unittest discover -s tests -p 'test_*.py'
```

## 本次诊断与验证（2026-11-03，Deadend / Frontier 更新）

只读检查用户运行中的旧图：89 节点（40 Endpoint、49 Junction），97 边；40 个端点中 32 个靠近 Unknown，因旧短分支保护策略保留下来。18 条路口间/自身边短于 0.35m。密集绿色节点并非 RViz 旧 Marker 堆积：细化只合并直接相邻的 Junction 像素，短 Connector 分隔的 Junction 会独立存在；家具边界和真实 Unknown 小洞也形成额外岔路。J53 的 0.20m 自环则对应完全 FREE 的 2×2 残留，已加入修正；139–136、132–137 附近有真实 Unknown 孔洞，不能靠随意填洞或按距离合并来抹除。

新增回归覆盖单个与相邻填满像素面的假环、真实孔洞保留、一次性 Endpoint 过滤、绿色路口锚点保留、纯环保留、隔墙最近点选择、完整绕行路径、Unknown/膨胀/对角穿角排除、前沿消失与异步旧结果丢弃、单格开门后连接刷新且几何变化累计不丢失。

完成记录：

- 在现有 `apexnav-ros2-jazzy` 容器中正常 `build/`、`install/` 重新编译 `plan_env` 和 `exploration_manager`，2 包通过；无需加载临时 overlay，无新增环境或模型下载。
- `ctest --test-dir build/plan_env --output-on-failure`：6/6 通过，包含 OpenCV/内置细化、Frontier 连接及真实 SDFMap2D 管理器测试。
- 原 Python 单元回归：50/50 通过。
- domain 94 的实际 APEXNAV ROS 启动：归一化深度场景生成 1 个 Junction、0 条 Junction–Junction 边，过滤 3 个 Endpoint，连接 1/1 个 Frontier；原 Frontier 继续显示，只有原控制器发布速度。采样 16.5ms、几何构建 3.8ms、连接 0.6ms（小型合成场景测量，不代表大型场景上限）。
- 地图无变化不重算、finish 保留图、reset 同时清空骨架及 Frontier 连接、第二场景独立构建均通过。
- 原连续和离散 APEXNAV 语义回归均通过：延迟 BLIP2 帧对应原位姿、前沿更新及旧语义消息在 reset 后丢弃。
- `git diff --check` 通过；编译仍有原依赖/弃用警告，无构建错误。

容器日志：`/tmp/skeleton_frontier_build_final.log`、`/tmp/skeleton_frontier_ros.log`、`/tmp/skeleton-ros-6ggko0zd/planner.log`、`/tmp/skeleton_frontier_python.log`、`/tmp/skeleton_frontier_semantic_continuous.log`、`/tmp/skeleton_frontier_semantic_discrete.log`。

没有重新运行完整 HM3D 场景或实车试验。最终构建前容器已退出，重启容器后更新了常用 install；不再需要额外 overlay。按上面的命令重新启动规划器和仿真即可观察新效果。

## 节点中心连接与 2m 合并更新

本次修改 `skeleton_graph.{h,cpp}`、`skeleton_frontier_links.{h,cpp}`、`skeleton_graph_manager.{h,cpp}`、四个 Skeleton 消息、`skeleton_graph.yaml` 和相关测试。原 Frontier、语义、建图、局部规划与控制模块未修改。

用户要求自行检查实际效果，因此本轮仅做编译和小型核心逻辑检查，不执行新的 Habitat 仿真、ROS 集成实验或全量测评。保留并更新测试脚本，供需要时手动运行。`plan_env`、`exploration_manager` 两包编译通过；两个小型核心程序通过前沿中心连线、2m 阈值、两点中点、多点平均、关联边更新、真实环和几何锚点检查。未运行本轮 ROS/Habitat 集成实验。构建日志：容器 `/tmp/skeleton_centers_merge_build.log`。

重启原 APEXNAV 规划器后生效（容器内）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py \
  planner:=APEXNAV mode:=continuous topology:=none skeleton:=true
```

RViz 继续使用 Skeleton Graph 和原 Frontier 显示；原仿真命令不变。`ros2 topic echo /skeleton_graph/graph --once --field frontier_links` 可查看每条连接的原质心、几何路径与到中心的显示路径。

## 前沿连线折返修正（2026-11-03）

原因是上一版 Dijkstra 从旧锚点求路，之后附加新中心和 Frontier 质心，产生非必要折返。本轮修改 `skeleton_frontier_links.{h,cpp}` 和管理器的实时线段检查：使用当前中心、限制接入在前沿中心附近、栅格可视简化、直接可达时直连、线段检查包含格子角点两侧，避免穿墙/穿角。消息仅更新注释，字段布局未变。

只做小型核心程序和编译检查：开放区域合并节点到 Frontier 为单条直线，7 格×0.1m 的连线长度为 0.7m；墙体绕行、Unknown/膨胀/穿角排除、阻塞中心拒绝等检查通过。未启动新的 ROS/Habitat 仿真实验。

现有规划器运行期间在 `build/skeleton_link_fix` 单独编译 plan_env（通过），再原子替换常用 `install/plan_env/lib/libplan_env_lib.so`，旧进程仍使用旧库。**无需额外 source overlay，重启规划器后加载新库**。构建日志：容器 `/tmp/skeleton_link_fix_build.log`。
