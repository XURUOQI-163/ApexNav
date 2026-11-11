# Skeleton Graph 全局路径规划

该模式在 APEXNAV 的前沿评分和目标选择完成后，用骨架图 Dijkstra 替换当前目标的全局路径。默认仍使用原来的二维 A*。它与旧的 `planner:=TOPOLOGY` 深度描述子模式独立。

## 启动

在 ROS2 容器内编译（本次已完成）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select plan_env exploration_manager --symlink-install \
  --parallel-workers 2 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

停止旧规划器，再启动骨架全局规划；ROS_DOMAIN_ID 与仿真端保持一致：

```bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py \
  planner:=APEXNAV mode:=continuous global_path:=SKELETON utm:=true
```

`utm:=true` 保留现有拓扑持久化功能；规划本身不依赖 UTM，可省略。选择 `SKELETON` 会自动开启骨架提取和前沿附着。Habitat、YOLOE、BLIP2 和 RViz 启动命令不变；Habitat 继续使用 `--mode continuous`、`--planner APEXNAV`（默认），不要选旧 `TOPOLOGY`。

与原算法对照时，只需重启规划器并改为 `global_path:=GRID`。离散模式也支持替换全局路径，但没有连续轨迹/MPC 执行链路。

## 调用关系与边界

1. 原 `chooseExplorationPolicy()` 保持语义评分、TSP、距离代价和目标选择逻辑，包括这些步骤使用的二维 A*；同时保留选中目标的原网格路径。
2. `ExplorationManager::applySelectedSkeletonPath()` 向 `SkeletonGraphManager::routeToFrontier()` 查询内存中的当前图，检查所选 Frontier 的实际成员签名与异步生成的附着是否一致。
3. `skeleton::shortestRoute()` 将当前位置连接至附近可直视的骨架节点或边内部。每条边按完整 Polyline 的实际弧长计费，Dijkstra 搜索并按行进方向拼接路径，保留转折点。
4. 路径终点为当前 Frontier 附着的安全接近位置。附着中面向 Unknown 前沿的展示关联线不用于通行。节点和所有路径段用当前 SDF 的 Free、Occupied、Unknown、Inflated 状态重新检查。
5. 连续 FSM 复用现有约 4m 路径前视、KinoAstar、GCopter 和 MPC。骨架折线提供局部目标引导，局部轨迹仍由原规划器生成，实际轨迹可偏离骨架以避障；并非强制 MPC 逐点跟踪原始折线。

Dijkstra 得到的是当前已验证拓扑及起终点连接中的最短路径，不等于整个自由空间内的最短连续曲线。当前实现不取消评分阶段的网格搜索，因此不保证整体规划耗时下降。

骨架未生成、没有有效前沿附着、起点接不上图、图不连通或当前地图阻断路径时，继续使用缓存的原二维 A* 路径。若原局部轨迹规划函数对骨架局部目标返回失败，则恢复网格路径并重试一次。两条路线均失败时，沿用原 FSM 失败处理。目标对象导航维持原行为；本次只替换 Frontier 探索路径。

## 配置和 RViz 检查

`src/planner/exploration_manager/config/skeleton_route.yaml`：

```yaml
/**:
  ros__parameters:
    skeleton_route:
      connection_radius: 2.0  # 米，当前位置接入可见骨架的搜索半径
```

骨架蓝线继续表示全图；已有 Exploration 显示中的 `next_path` 红线表示当前实际选择的全局路径。局部优化后的轨迹使用原有轨迹显示。

终端日志：

- `[GlobalPath] SKELETON: edges=... length=... points=...`：本轮使用骨架路径。
- `[GlobalPath] GRID fallback: ...`：本轮使用原二维 A*，后面是原因。
- `FRONTIER_ATTACHMENT_NOT_READY`：目标与当前骨架附着尚未匹配，通常需要等待骨架异步更新。
- `NO_LOCAL_GRAPH_CONNECTION` / `DISCONNECTED_GRAPH`：无法接入骨架或骨架不连通。
- `INVALID_FRONTIER_ATTACHMENT`：前沿接近路径与当前地图或节点几何不一致。
- `local trajectory failed`：局部轨迹规划报告失败后恢复网格路线。

## 验证

```bash
ROS_DOMAIN_ID=95 ctest --test-dir build/plan_env --output-on-failure
```

本次 `plan_env`、`exploration_manager` 编译通过；7 项 CTest 全部通过。新增测试覆盖真实折线长度、反向边、平行边、边内部接入、闭环、障碍/Unknown、断图和失败原因。隔离 ROS_DOMAIN_ID=96 启动检查确认 `SKELETON` 参数传递、自动开启骨架及原轨迹节点正常初始化，随后主动中断退出。没有运行 Habitat 完整探索实验。

核心新增文件为 `plan_env/{include/plan_env,src}/skeleton_route.*`、`plan_env/tests/skeleton_route_test.cpp` 和 `exploration_manager/config/skeleton_route.yaml`。接入位置为 SkeletonGraphManager、SDFMap2D/MapROS 只读访问接口、ExplorationManager、两个 FSM 和 simulation/algorithm 启动文件。
