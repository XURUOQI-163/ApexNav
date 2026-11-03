# 拓扑探索闭环设计（2026-10-30）

授权范围：用户要求在当前已修改 ROS2 项目继续实现拓扑全局 A*、约 6m 滚动局部地图、局部避障和 MPC 连续执行。保留 APEXNAV 对照，新增代码和设计集中在本目录所属包；不接实车、不加入语义。

## 选型与复用

原 SDFMap2D 的栅格原点固定，initMap 自动构造语义/前沿/ROS 接口；KinoAstar 要求终点在地图内，GCopter 依赖 ESDF。选择用户已授权的局部栅格 A* 回退路线，避免把全局拓扑终点直接送到有限窗口外的轨迹优化器。另一种整体改写 KinoAstar/GCopter 窗口适配会影响对照，暂不采用。

新增 topology_exploration ROS2 包：RollingLocalMap 包装共用 SDFMap2D，GraphPlanner 共用 DepthDescriptor/TopologyGraph，LocalPlanner 生成已验证参考，TopologyExplorer 维护状态与唯一 /cmd_vel 发布。共用现有 MPC 源码和 OSQP 求解器，通过库导出使用；新模式启用严格约束及求解状态检查。旧模式启动入口与算法保持默认。

## 地图和深度

SDFMap2D::initMap 增加默认开启的 ROS/语义附件参数；新模式关闭附件，只有固定 6×6m、0.05m 分辨率栅格。RollingLocalMap 是 friend 包装，在整数格平移时拷贝重叠区域、丢弃窗口外数据，新进入区域 Unknown，重置射线缓存及距离场。不创建全局占据副本。

复用原投影公式/射线更新；深度与相机位姿 ExactTime 同步，保留浮点有效性，零/NaN/超量程不能清障。共享深度投影工具供 MapROS 和新模式使用。地图外显式 Unknown；原地图模式不改其旧返回约定。

Limo Pro 手册尺寸 0.322×0.220m；采用外接圆半径 hypot(length,width)/2 + 0.05m 安全余量（约0.245m），长度/宽度/余量可配置。有效占据检查覆盖足迹所交格子，Unknown 同样不可通行。初始站立足迹可由无碰撞初始位姿初始化为已知自由，已有障碍绝不覆盖；不清空周围未知区域。只在已验证足迹上允许原地观察。定位跳变、深度/定位失效停车。新模式中的有效深度命中立即形成占据证据；自由更新仍累计概率。

## 图、边和搜索

保留 FRONTIER/WAYPOINT 及方向图像。拓扑图更新增加可选局部观测范围，默认行为不变。远端节点/边不会因移出窗口消失；窗口内真实障碍撤销有效性。历史自由不能授权当前未知区的局部运动。

每条边保存验证状态、验证时间、可执行几何折线及路径长度代价；直线可视边的初始代价是验证线段长度，实际行驶后的边更新为里程折线长度。禁止用直线距离冒充绕障路径代价。失效边不能参与 A*。

GraphPlanner 对每个 ACTIVE/冷却已过 FRONTIER 运行 A*（欧氏下界），总路径代价最小者获选。只允许 WAYPOINT 作为中间节点，FRONTIER 仅终点。机器人不在节点时先用局部 A* 验证到附近 WAYPOINT 的连接，接入代价计入总价。局部可达新节点有必要时生成 WAYPOINT。全局记忆保持 ID，转换、合并/移位同步更新边。

失败目标进入 30 秒冷却并计次；同 ID 失败 3 次为 BLOCKED，episode 复位后恢复，防止每个周期重复选同一点。局部障碍可封锁相关边；未知/窗口截断不等同于永久拓扑不可达。

## 执行与停车

拓扑节点序列经边折线选择当前窗口的局部引导点；依次通过折线顶点，避免局部捷径跳过尚未推进的图路径段。LocalPlanner 8 邻域 A*，禁止对角穿角；所有连接段做足迹检查。终点超出已知可通行区时返回向目标推进的安全前缀，末端速度降到零；无安全前缀停车并重新感知/重规划。

MPC 采用现有 xdot=v cos(yaw), ydot=v sin(yaw), yawdot=w 的差速等效模型，输出 Twist.linear.x/angular.z。默认 vmax=0.4m/s、wmax=1rad/s、amax=0.5m/s²、alphamax=1rad/s²，可配置；这些为仿真保守设定，不声称厂家加速度额定值。20Hz 控制、5Hz 局部重规划；首控制量和序列相邻控制量均有增量约束。外部执行守卫再限制速度、加速度，并检查最终限幅命令及其制动过程的扫掠路径；不安全、QP 失败/非有限、无路径、深度/里程计超时、定位突跳立即发零（紧急停车优先于平滑减速）。

状态 WAIT_DATA→OBSERVE→PLAN→TRACK→ARRIVE/REPLAN；目标失败转 COOLDOWN 并选其他目标；没有可达目标时停车等待，不发物体找到信号。初始与到达后在足迹安全且数据新鲜时用 MPC 原地转向重新观测。控制器只发 /cmd_vel；TOPOLOGY 模式不启动旧 exploration_node/traj_server，因此无双发布。

## 集成

simulation.launch.py 新增 planner:=APEXNAV|TOPOLOGY，默认 APEXNAV；TOPOLOGY 要求 mode:=continuous，包含独立包 launch。旧 topology:=sparse 观察开关继续可用于 APEXNAV 对照。

Habitat CLI 同步新增 --planner，TOPOLOGY 关闭视觉/LLM进程和仿真侧强制初始旋转，完全跟随 /cmd_vel；不按旧语义目标结束探索，保留用户配置的仿真时间预算。新节点实现现有 /exploration/reset、/traj_server/reset、/traj_server/stop 契约，reset 清空窗口、图、目标、路径、MPC 历史并拒绝旧消息。无模型服务依赖。

RViz 显示滚动 OccupancyGrid、绿/红节点、蓝边、全局节点路径和局部安全前缀；状态话题输出目标、原因、路径代价、失效/冷却状态及固定栅格容量。

## 验证

确定性测试：滚动平移保留重叠/外部 Unknown/固定容量；矩形机器人膨胀；A* 最短代价含接入/阻断边/Frontier不作中继；局部绕墙/窄缝/对角穿角/窗口截断安全前缀；失败冷却；首输入加速度/QP失败清零；深度/位姿超时与突跳停车；reset旧帧。

域94 ROS合成闭环：实际节点收到深度和位姿，集成命令积分成里程计；验证仅一个 cmd_vel 发布者，机器人朝目标推进、到达后转换、障碍后停车/重规划、停深度后零速度、窗口随位置平移且容量不变。回归 APEXNAV 旧测试和上一阶段拓扑观察测试。真实 HM3D 长程评测由用户进行。

参考尺寸：https://indrorobotics.ca/wp-content/uploads/2024/07/limo-pro-user-manual-en.pdf （厂家手册第1.3节，由经销商托管）。
