# 可选稀疏几何拓扑图

本模块是同进程的几何观察模块。`topology:=sparse` 开启建图，默认 `topology:=none` 关闭。
原有二维栅格、前沿语义记忆、目标检测、探索状态机和运动控制继续工作；新图不发布运动目标、速度或停止信号。

## 启动

在原来的规划器终端中，停止旧规划器，然后运行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py \
  mode:=continuous topology:=sparse \
  topology_sector_degrees:=30.0 topology_max_distance:=5.0
```

离散模式改为 `mode:=discrete`。Habitat 的 `--mode` 必须与规划器一致。
仿真、YOLOE、CLIP/BLIP2 的启动命令保持原样，无需模型或数据下载。
使用原来的 RViz 配置即可看到 **Sparse topology**：绿色 WAYPOINT、红色 FRONTIER、蓝色连边及节点编号。
若 RViz 已经打开，需要重新加载配置，或自行添加 MarkerArray `/sparse_topology/markers`，Fixed Frame 为 `world`。

查看实时图结构（包含节点、描述子、图片元数据、边及权重）：

```bash
ros2 topic echo /sparse_topology/state --once
```

`duplicates`/日志中的 `duplicate_rejections` 是已拒绝的重复候选次数，不是图中存在的重复节点数。
节点 ID 单调递增，有删除时不连续；每次 episode reset 从头开始。

## 处理流程

1. `MapROS` 按原来的精确时间戳同步深度与相机位姿。可选 observer 在原有逐像素投影中取得世界坐标点，不订阅/反解 PointCloud2。保持原有建图障碍高度范围。
2. `DepthDescriptor` 默认有 12 个世界坐标对齐的扇区，30° 一个。每个扇区存最近有效障碍距离、保守可见距离、观测状态、时间及角度覆盖标记。0° 扇区中心是世界 +X。
3. 前向视场之外保持 UNOBSERVED；零、NaN、非有效量程的深度记为 INVALID，不能当作 5m 空地。每个扇区划分 6 个角度子区间，全部得到有效观测才用于覆盖判定，避免一个有效像素清空整个扇区。


4. 原地改变朝向可补齐不同方向。同一位置的测量才能合并；发生平移时保留新描述子的真实观测原点，不能把异地测量拼成伪全景。节点内同时保留按朝向分槽的图像。？？？


5. 从连续开放扇区、相邻有效扇区的距离突变生成候选，沿观测射线回退到已知自由空间。候选及连接线的圆形占用范围所交栅格必须全部为 FREE，且候选中心不能处于膨胀障碍；未知、障碍和地图外均拒绝。
6. 以节点间距和历史 WAYPOINT 描述子的空间覆盖关系去重。未访问 FRONTIER 的描述子来自其生成位置，显式标记 `measured_at_node=false`，不能用作新的覆盖证据。
7. 距 FRONTIER 0.4m 内且局部连通时，保留 ID、转换为 WAYPOINT，位置改为实际到达位置并换成当地真实描述子/图片。图不负责挑选运动目标，因此这里判断的是“实际接近任意候选点”。旧规划路径不经过候选点时，每隔约 1.5m 记录一个 WAYPOINT，保持对实际探索过程的记录。
8. 任一端真实 WAYPOINT 描述子支持可视连通，且当前栅格上的连接走廊可通行，才建立无向边。权重为世界坐标平面欧氏距离。地图变化会删除无效节点/边；到达导致节点移动时重新计算边权。重复访问复用 WAYPOINT ID。

拓扑 FRONTIER 是深度开放/突变生成的可通行候选位置，与旧栅格算法的前沿格子、语义前沿中心并非一一对应。
当前阶段的连边使用经过栅格复核的可视连接；不推测穿墙捷径，也没有增加沿历史曲折轨迹的边。

## 图像与存储

图像、深度、位姿使用相同采集时间戳。每个节点最多保存一个扇区一个视向槽（默认 12 槽）的 JPEG，宽度最多 320px，质量 75；同一帧可由多个节点共享。
每张图片记录采集时间、真实相机位置和四元数/朝向。FRONTIER 的图片在其来源位置拍摄，不是假设在尚未到达的位置拍摄。
内存中的 RGB 等待窗口及位姿窗口各最多 20 帧。未匹配到的新图像不会清除已保存图片；待匹配时间戳最多保留 20 项，延迟图像可以补齐，乱序旧图不会覆盖新图。每张已保存图片保留它自己的采集时间与位姿，可能早于最新深度观测。尚未收到图片的视向不会出现在 `images` 中。

图存放在 `SparseTopologyObserver::graph_` 内存中，核心类型是 `TopologyGraph`。本阶段不自动写磁盘；`/sparse_topology/state` 输出图片元数据，不输出 JPEG 本体。
原有 `/exploration/reset` 重建地图时同时清空图、图像和 RViz 标记，过期传感器帧无法恢复上一轮图。

## 参数与输入约定

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `sparse_topology.enabled` | false | 是否实例化 observer |
| `sparse_topology.sector_degrees` | 30 | 扇区角，5–90° 且整除 360° |
| `sparse_topology.max_distance` | 5 | 描述子有效水平距离上限，m |
| `sparse_topology.open_distance` | 0.9 × 上限 | 开放扇区判定阈值，m |
| `sparse_topology.candidate_step` | min(3.5, 0.7 × 上限) | 候选投放距离上限，m |
| `sparse_topology.jump_distance` | 1 | 相邻扇区突变阈值，m |
| `sparse_topology.reach_radius` | 0.4 | 到达/重访范围，m |
| `sparse_topology.duplicate_radius` | 0.7 | 候选距离去重范围，m |
| `sparse_topology.waypoint_spacing` | 1.5 | 未到达候选时的轨迹锚点间距，m |
| `sparse_topology.clearance` | 0.18 | 对未知格也生效的占用半径，m |
| `sparse_topology.update_hz` | 2 | 描述子/图的更新频率上限 |
| `sparse_topology.max_depth_is_free` | true | 饱和深度是否表示量程内自由观测 |

深度沿用当前 Habitat 接口：32FC1 的 [0,1] 归一化深度，或对应的 16UC1；不是以米为单位的通用 ROS 32FC1 深度接口。公制反投影复用 MapROS 的内参和深度量程。
默认将有效的最大量程返回视为量程内自由，只适用于当前模拟器的输入约定。若改接“最大值代表无回波/无效”的传感器，应设 `max_depth_is_free=false`。
实际有效范围同时受传感器量程限制，调大 `max_distance` 不会制造传感器未看到的空间。
状态值：`0=UNOBSERVED`、`1=INVALID`、`2=FREE`、`3=OBSTACLE`；无有效距离在 JSON 中表示为 -1。

## 参考及验证

参考 [Palm-sized Omnidirectional Vision-Based UAV Exploration with Sparse Topological Map Guidance](https://arxiv.org/html/2605.07275v1) 的描述子、开放/突变候选和历史覆盖思路。本实现针对前向深度和地面机器人重新实现，未使用作者未公开的源码，也不复现其无人机规划/运动控制。

核心测试 `tests/sparse_topology_test.cpp` 验证前向/背向区分、无效深度、部分扇区、旋转/平移、开放与突变候选、去重、访问转换、重访、边权、未知格占用范围、动态障碍、图片匹配和重置。
ROS 测试 `tests/ros2_sparse_topology.py` 在隔离域 94 中给真实 C++ 建图节点发送合成前向深度/RGB/位姿，验证逐向补全、稳定节点数、实际重复数、图像时间戳、标记颜色、到达转换和 episode reset。
合成测试的节点数量只对应测试输入，不能代表 HM3D 场景的实际节点数量或导航性能。

```bash
ctest --test-dir build/plan_env --output-on-failure
source /opt/apexnav_ros2_py312/bin/activate
PYTHONPATH=.:"$PYTHONPATH" python tests/ros2_sparse_topology.py
```

实际验证记录（2026-10-29）：

- Jazzy 中 `plan_env`、`path_searching`、`trajectory_manager`、`exploration_manager` 编译成功。
- CTest：稀疏拓扑与原有目标多帧置信度融合两项均通过。
- 原有 Python 单元测试：50 项通过。
- 新模式关闭时，原有 ROS 连续模式语义测试通过：延迟 BLIP2 分数按原图位姿融合一次，前沿语义/地图/路径在重置后清空，旧结果被拒绝。
- 真实 ROS 合成测试通过实际 `simulation.launch.py topology:=sparse` 入口运行；原地转向并重复观测后 1 个 WAYPOINT、6 个 FRONTIER、6 条边，实际重复节点为 0；本次运行拒绝重复候选 216 次。
- 模拟到达后 2 个 WAYPOINT、5 个 FRONTIER、6 条边，节点 ID 保留，边权与新位置一致。
- 精确时间戳图像匹配、RViz 节点颜色/编号/边、图重置及过期帧拒绝均通过。
- 核心回归额外覆盖 RGB 丢帧、延迟和乱序，以及前向运动的历史可见连边和未知格占用范围边界情况。

以上为逻辑及 ROS 接入验证，尚未执行完整 HM3D 场景探索实验，也未进行导航性能对比。
