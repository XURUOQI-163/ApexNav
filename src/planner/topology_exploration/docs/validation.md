# 验证记录（2026-10-30）

环境：现有 `apexnav-ros2-jazzy` 容器，ROS2 Jazzy，现有 OSQP/OsqpEigen 和 Habitat Python 3.12 环境。ROS 集成测试只使用 domain 94，无实车连接，无模型或场景下载。

| 检查 | 结果 |
|---|---|
| `colcon build`：plan_env、path_searching、trajectory_manager、topology_exploration、exploration_manager | 通过；最后同步位姿修正后再次构建 topology_exploration 通过 |
| 新包 CTest | 4/4 通过（滚动地图、图 A*、局部 A*/守卫、MPC） |
| plan_env CTest | 2/2 通过（原稀疏拓扑、目标多帧融合） |
| 原 Python 单元测试 | 50/50 通过 |
| 新模式 CLI 单元测试 | 3/3 通过 |
| 原自定义目标受控 runner | 离散/连续及视频分支通过 |
| 新几何模式受控 runner | 无视觉进程、无语义目标/标签/触发、初始运动来自 cmd_vel，均通过 |
| Python 语法、YAML/RViz 解析、`git diff --check` | 通过 |

最终合成 ROS 闭环运行的实测输出：

```text
PASS ROS closed loop: traveled=4.00 converted=2 waypoints=3 frontiers=10 edges=15 cells=14400
PASS depth timeout: zero v,w
PASS invalid depth cannot refresh watchdog
PASS odometry timeout: zero v,w
PASS fresh close obstacle: immediate occupied cells and zero v,w
PASS reset clears graph, rolling map, target and command
```

测试直接积分节点发布的 v/w 生成里程计，再发布带相同时间戳的前向深度、相机位姿和图像。检查只有一个速度发布者，速度/正常增量满足限制，机器人完成两次前沿转换，窗口原点随位置变化且始终 14,400 格。选择最小路径代价的前沿允许回访附近方向，因此检查累计里程，而不是强制径向远离出生点。节点/边数量受实际 ROS 回调时序影响，不是固定性能指标。

原观察模式 ROS 回归：稳定 W1/F6/E6，实际重复节点 0；到达后 W2/F5/E6，原 ID 转换、方向图像时间戳及 reset 拒绝旧帧通过。原连续 APEXNAV 的语义对齐回归：延迟 BLIP2 分数更新原帧对应前沿、每格融合、重置清空地图/前沿/标记并拒绝旧分数，均通过。

重点回归用例包括：

- 连续 30 次自由观测之后，新障碍第一次命中就成为占据。修正前该测试失败，修正后通过。
- 已走过的 3.1m 绕行边在窗口移走后保留路径与代价；中间段出现障碍后撤销验证，Unknown 不恢复验证。
- 完整 360° 深度描述子、6m 局部窗口下，连续八次前沿推进至 x=16.42m、9 个 WAYPOINT，避免历史覆盖把候选全部删除。
- 按待经过的折线顶点更新路径进度，避免局部捷径跳过顶点后在窗口移动时反向回走。
- MPC 首步增量、跨 ±π 转向、空参考和 OSQP 拒绝不一致控制约束时归零。

独立代码审查发现并复核修正了新障碍累计延迟、全景覆盖导致候选耗尽、路径进度被捷径跳过三个问题；复核无遗留阻塞项。

以上为最初实现的验证记录；当时的合成场景没有覆盖下面的家具遮挡问题。未进行 HM3D 长程性能评测，也不代表实车控制性能。

## 家具场景候选恢复（2026-10-30）

真实 episode 550 的旧版本复现：初始转一圈后 W1/F0/E0，位置无平移。固定开放门槛为量程的 90%（4.5m），12 个粗扇区没有任何一个达到门槛；唯一深度突变候选端点可通行，但直线安全前缀仅 0.125m，被 0.8m 最小候选距离过滤。同一局部地图的 A* 实际能给出约 2.13m 的安全绕行路径。

修复内容：开放门槛独立为 1.2m；保留粗扇区最近障碍并新增每扇区六个细方向用于候选选择；用现有局部 A* 验证绕行边并保存折线路径代价；添加状态与候选过滤诊断。审查补充修复了 fine bin 历史最短距离无法被新观测更新，以及重复候选阻止失效边恢复的问题。现有前沿 ID 在重新验证时保留。

新增 `frontier_generation_test` 在修复前分别复现候选缺失、旧短距离无法恢复和失效边无法恢复，修复后通过；还检查无效/旧帧拒绝、实际绕行长度、回调返回穿障碍路径的拒绝、新障碍下的替代路径、重复过滤和 reset。原稀疏观察模式的单元测试继续通过。

最终实际场景测试命令：

```bash
PYTHONPATH=.:$PYTHONPATH python src/planner/topology_exploration/tests/ros2_hm3d_frontier.py
```

现有 Jazzy 容器，domain 94，HM3Dv2 索引 550、episode_id 19，场景 `00873-bxsVRursffK`，1500 ticks（50s 仿真），原 NavMesh、`allow_sliding=true`，未使用视觉/语义服务。结果：

| 实测指标 | 结果 |
|---|---:|
| 累计移动距离 | 3.3081m |
| 距出生点最大位移 | 1.9597m |
| FRONTIER → WAYPOINT 转换 | 4 次 |
| 采样到的 WAYPOINT / FRONTIER / 边峰值 | 4 / 6 / 15 |
| 累计新建节点（含后续删除的节点） | 17 |
| 累计绕行边验证/重建 | 6 |
| 同时存在的重复节点对（距离小于 0.35m） | 0 |

原始日志位于容器 `/tmp/hm3d_frontier_jna6csxk/`，包含 `planner.log`、`simulator.log`、`states.json`、`positions.json`、`result.json`。状态以 2Hz 采样，数量受回调时序影响。该测试证明这个场景能从初始旋转进入运动并持续转换前沿，不是全场景覆盖率或最优参数评测。

最终五包重新编译通过；新包 CTest 5/5、plan_env CTest 2/2、原 Python 50/50、新 CLI 3/3、几何 runner 均通过。YAML、Python 语法和 `git diff --check` 通过。独立审查复核两个恢复问题后无遗留 Critical/Important。

最终合成 ROS 闭环：累计 4.00m、转换 2 次，W3/F17/E25，地图始终 14,400 格；深度超时、无效深度不能刷新看门狗、里程计超时、新障碍停车、reset 清除图/地图/目标/命令全部通过。原稀疏观察模式仍为 W1/F6/E6、实际重复节点 0；到达后 W2/F5/E6，原 ID 转换及旧时间戳拒绝通过。

本轮修改文件（路径相对于 `src/planner`）：

| 文件 | 内容 |
|---|---|
| `plan_env/include/plan_env/depth_descriptor.h` | 细方向采样及新旧观测更新规则 |
| `plan_env/include/plan_env/topology_graph.h`、`plan_env/src/topology_graph.cpp` | 候选配置、过滤诊断、完整路径验证和失效边恢复 |
| `topology_exploration/src/explorer_node.cpp`、`include/topology_exploration/explorer_node.h` | 参数接入、复用 LocalPlanner、日志状态 |
| `topology_exploration/src/visualization.cpp`、`config/topology.yaml` | 终端/状态 JSON 诊断及默认参数 |
| `topology_exploration/tests/frontier_generation_test.cpp`、`CMakeLists.txt` | 新 C++ 回归用例及构建注册 |
| `topology_exploration/tests/ros2_hm3d_frontier.py` | 使用已有场景的真实 Habitat 回归入口 |
| `topology_exploration/README.md`、`docs/frontier_recovery_plan.md`、本文件 | 使用说明、修复计划与验证记录 |

最终原连续 APEXNAV 语义对齐回归也通过：延迟 BLIP2 分数只更新对应原帧前沿，分数不重复融合；reset 清空地图、前沿和标记，拒绝旧分数，后续场景独立。

## 快照持久化与启动统一重置（2026-11-01，11-02 文件复核）

按用户要求，仿真退出不再 reset。下一轮启动使用已有 `/exploration/reset` 统一清空地图、前沿、控制状态和磁盘快照；没有新增单独的磁盘重置服务。连续、离散仿真及原 APEXNAV 状态机的结束处理同步调整为停车并保留地图。多 episode 运行在下一 episode 开始前重置。

新增 `include/topology_exploration/snapshot.h`、`src/snapshot.cpp`，在 `explorer_node` 中接入单任务后台保存与显式退出刷新；更新 CMake、配置、README 和项目 `.gitignore`。仿真生命周期修改位于 `habitat2ros/episode_reset.py`、`habitat_vel_control.py`、`habitat_evaluation.py` 及两个原探索 FSM。字段和使用方式见 [snapshot.md](snapshot.md)。

验证结果：

- 编译通过；topology_exploration CTest 7/7、plan_env CTest 2/2；原 Python 用例 50/50、CLI 用例 3/3。
- C++ 写入器及实际 JSON 检查覆盖完整节点/边、12/72 方向、非有限距离 null、图片引用、过期图片清理和写入失败保留上一份完整快照。
- ROS domain 94 隔离测试验证正常停止保留图与 30 张 JPEG；图片时间戳、相机位姿和解码像素与发布数据相符。停止后迟到的 RGB 仍会保存；上一次后台写入失败不阻止最后一次刷新。Ctrl+C 保留文件，规划器重启不清空，下轮仿真 reset 清空上一轮节点与图片。
- 连续 runner 的正常结束和 KeyboardInterrupt 均只在启动 reset；几何模式及自定义目标回归通过。原 APEXNAV 连续、离散语义对齐回归均验证结束保留地图/路径，下一轮 reset 清除并拒绝旧帧。
- 真实 HM3D 索引 550、1500 ticks：累计移动 3.4223m，最大位移 1.9444m，4 次前沿转换，节点峰值 WAYPOINT 4 / FRONTIER 8，边峰值 18，重复节点对 0。
- 11-02 从已停止容器取出真实运行留下的文件复核：最终图包含 3 个 WAYPOINT、8 个 FRONTIER、18 条边和 58 张 JPEG。所有边端点存在、节点 ID 唯一、图片引用与文件集合一致；58 张图片全部成功解码为 320×240，文件大小、时间戳、相机位姿及单位四元数检查通过。

真实运行文件位于测试容器 `/tmp/hm3d_frontier_27b2_yja/topology_snapshot/`，ROS 生命周期测试日志为 `/tmp/topology_snapshot_ros_final.log`。测试全部使用临时保存目录，没有覆盖项目根目录中的用户快照。该短程回归验证保存和生命周期，不代表完整场景探索性能。
