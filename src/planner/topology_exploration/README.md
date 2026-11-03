# 独立几何拓扑探索模式

`APEXNAV` 保留当前长期二维地图、前沿语义记忆、CLIP/BLIP2 和原导航链路。`TOPOLOGY` 使用稀疏图 A* → 滚动局部地图 A* → 共用 MPC，当前只做几何探索，选择**接入代价 + 图路径代价最小的可达 FRONTIER**。两个模式分别运行。

## 编译与启动

代码已经在现有 `apexnav-ros2-jazzy` 容器编译，不需要新增模型下载。以后修改源码后，在容器内执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select plan_env path_searching trajectory_manager topology_exploration exploration_manager --executor sequential --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

先结束旧规划器及 `traj_server` 的 launch，再启动新模式。以下每个终端都在同一容器内，并设置相同的 `ROS_DOMAIN_ID`。

**终端一：规划器**

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py planner:=TOPOLOGY mode:=continuous
```

**终端二：Habitat**（纯几何模式不需要启动 YOLOE、CLIP 或 BLIP2 服务）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
export ROS_DOMAIN_ID=31
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
python habitat_evaluation.py --mode continuous --planner TOPOLOGY \
  --dataset hm3dv2 test_epi_num=550 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets \
  habitat.simulator.habitat_sim_v0.allow_sliding=true
```

`test_epi_num` 在当前连续 runner 中是 episode 索引；新模式仍使用现有单场景时间预算。`--planner TOPOLOGY` 必须配合 `--mode continuous`，不接受语义 `--target`。相机 EGL/GPU 环境继续使用你已配置好的容器设置。

**终端三：RViz**

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 ros2 launch exploration_manager rviz.launch.py
```

勾选 `Sparse topology`、`Topology rolling local map`、`Topology global route`、`Topology local safe path`；固定坐标系是 `world`。绿色 WAYPOINT、红色 FRONTIER、蓝色已验证边、编号；失败/冷却节点用橙色。新模式不发布旧的全局占据/语义地图，旧显示项可以取消勾选。

切回对照：规划器用 `planner:=APEXNAV mode:=continuous`，仿真用 `--planner APEXNAV --mode continuous --semantic-model blip2`（或 `clip`），并按原方式运行视觉服务。省略 `--planner` / `planner:=...` 的默认值仍是 `APEXNAV`。

## 结构与数据

| 文件 | 职责 |
|---|---|
| `src/rolling_local_map.cpp` | 包装共用 SDFMap2D，整数格滚动、射线更新、未知区和足迹检查 |
| `src/graph_planner.cpp` | 已验证图边的 A*，机器人接入代价，失败冷却/封锁 |
| `src/local_planner.cpp` | 局部八邻域 A*、防对角穿角、按边折线顺序执行安全前缀 |
| `src/explorer_node.cpp` | 深度/位姿精确同步、状态机、MPC、单一速度发布者、看门狗与重置 |
| `include/topology_exploration/controller_guard.h` | 最终 v/w 增量限幅与曲线行驶、制动扫掠检查 |
| `src/visualization.cpp` | RViz 栅格/节点/边/路径和状态 |
| `config/topology.yaml` | 地图尺寸、底盘尺寸、速度/加速度、超时等配置 |

共用 `plan_env` 的深度投影、射线工具、`DepthDescriptor/TopologyGraph` 与 `trajectory_manager` 的 MPC。旧 KinoAstar 要求目标在地图内，GCopter 依赖 ESDF；本阶段使用已授权的局部栅格 A* 路线，未重写旧规划器。

局部占据窗口默认 6×6m，0.05m 分辨率，即 **120×120 = 14,400 格**。滚动只保留重叠区域，窗口之外和新进入部分是 Unknown；不另存全局占据地图。拓扑图、边路径和节点方向图像是长期记忆，因此图总内存仍会随探索增长。

边存储两端 ID、验证时间、有效性、路径折线及长度代价。尚未走过的边可以是已验证的安全直线，也可以是局部 A* 验证的绕行折线，代价均为完整路径长度；走过后用实际里程折线长度更新。远端图边不会仅因移出窗口而删除；新观察到的障碍会撤销相关边验证，未重新验证的边不能用于图 A*。历史边不能授权当前 Unknown 区域的局部运动。

候选生成保留 30° 扇区的最近障碍距离，同时记录其中六个细方向（默认每个 5°）的有效距离、状态和时间。TOPOLOGY 使用细方向中的连续开口（默认至少 10°）和深度突变选择候选；未知或无效深度不算开放方向。`sparse_topology.open_distance: 1.2` 是候选开放距离门槛，与 `max_distance: 5.0` 的描述子量程分开。`candidate_step: 2.2`、`min_candidate_distance: 0.8` 限制候选距离，最终仍以局部地图整条路径的底盘足迹检查为准。直线被家具挡住时，先尝试已知自由区内的 A* 绕行；不会仅因候选点本身自由就连边。配置在 `config/topology.yaml`，修改后重新编译该包并重启规划器。

近处候选是下一次感知的位置，其中心已被看见不等于周围已探索完成。新模式使用 `candidate_step + footprint_radius` 的邻域覆盖检查，避免 5m 描述子把受小窗口限制的全部近处候选删除；旧观察模式仍按原来的点覆盖规则。

初始化与到达 FRONTIER 后先在安全足迹内原地转向补全前向相机观测，再继续搜索。前沿到达后保留 ID 转为 WAYPOINT。失败目标冷却 30 秒；同 ID 失败 3 次封锁到 episode 重置。没有可达目标时停车等待新观测，不发送“找到物体”的完成信号。

## 控制与参数

默认 `v ≤ 0.4m/s`、`|w| ≤ 1rad/s`、`|Δv| ≤ 0.5Δt`、`|Δw| ≤ 1.0Δt`，20Hz 控制，约 5Hz 局部重规划。差速等效模型为 `xdot=v*cos(yaw), ydot=v*sin(yaw), yawdot=w`。MPC 必须启用严格约束，控制周期固定 0.05 秒、delay_num 为 0；这些是本阶段实现约束。

足迹使用 `hypot(0.322, 0.220)/2 + 0.05 ≈ 0.245m` 的外接圆，包含安全余量，相当于按足迹对障碍及 Unknown 做碰撞膨胀。尺寸来自 [Limo Pro 厂家手册第 1.3 节](https://indrorobotics.ca/wp-content/uploads/2024/07/limo-pro-user-manual-en.pdf)。速度/加速度是本项目的保守配置，不是厂家额定加速度。初始位置需已有有效、无碰撞的 Habitat 出生位姿；只将机器人当前足迹初始化为自由。

碰撞高度带默认世界地面以上 0.05–1.25m，可在 YAML 修改；这比原 Habitat HM3D 0.88m 高、0.18m 半径的代理更保守，不会修改原仿真代理和 NavMesh。若改变相机内参、深度归一化、地面坐标或代理尺寸，需要同步配置。新节点深度输入沿用本项目归一化 `32FC1`/`16UC1` 的约定。

深度命中立即参与障碍判断；自由更新仍累计证据。默认深度超过 1秒、里程计超过 0.5秒、无安全路径、定位突跳、非有限数据或 MPC 失败时发零速。安全守卫检查**最终限幅后的**命令及其制动轨迹。紧急零速优先于平滑减速。深度无效帧不能刷新看门狗。定位突跳须重置；出现第二个 `/cmd_vel` 发布者时新节点停车并报告。

## 查看状态和重置

```bash
ros2 topic echo /topology/state --once
ros2 topic echo /cmd_vel
ros2 service call /exploration/reset std_srvs/srv/Trigger '{}'
```

状态 JSON 包含阶段、原因、目标 ID、路线代价、节点坐标、节点数量、地图容量/原点和失败记录。`candidate_diagnostics` 是最近一次图更新的开放方向数、开放/突变候选数、接受数、绕行数、既有节点连接恢复数（`reconnected`），以及距离太近、足迹不安全、无路径、重复、历史覆盖等过滤计数；离开当前 WAYPOINT 后 `at_anchor=false`，该次不生成候选。`created_total`、`detours_total`、`duplicate_rejections_total` 是本 episode 的累计值（绕行边重建也计入 `detours_total`），重复过滤次数不是图中实际重复节点数。终端打印状态变化和每 5 秒一次的诊断摘要。完整图和方向图片自动保存到项目根目录 `topology_snapshot/`，详情见 [快照格式与生命周期](docs/snapshot.md)。仿真启动和切换到下一 episode 时通过原有 reset 握手统一清除内存及上一轮磁盘数据；正常退出、Ctrl+C 只停车并保留地图和快照。手动调用 reset 同样会清空它们，因此事后检查时不要调用 reset。

## 验证入口

本次编译与测试结果见 [验证记录](docs/validation.md)。

在源码根目录、ROS 与 workspace 已 source 后：

```bash
ctest --test-dir build/topology_exploration --output-on-failure
ctest --test-dir build/plan_env --output-on-failure
source /opt/apexnav_ros2_py312/bin/activate
PYTHONPATH=.:$PYTHONPATH python src/planner/topology_exploration/tests/ros2_closed_loop.py
# 已有 HM3D 数据集的实际 episode 550 回归，约 50 秒仿真时间：
PYTHONPATH=.:$PYTHONPATH python src/planner/topology_exploration/tests/ros2_hm3d_frontier.py
```

以上两项 ROS 测试顺序运行，均使用独立 ROS domain 94，不需要视觉服务。合成闭环检查 `/cmd_vel` 积分运动、节点增长、窗口滚动、障碍停车、深度/定位超时和重置。实际 HM3D 回归要求机器人产生超过 1m 的净位移、生成有边连接的前沿并至少转换一个 WAYPOINT，日志保存在测试打印的 `/tmp/hm3d_frontier_*` 目录。它们不代表整个 HM3D 的探索覆盖率或真实底盘性能。
