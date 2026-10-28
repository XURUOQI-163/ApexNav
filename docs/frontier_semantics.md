# 当前前沿格子的稀疏语义记忆

语义长期存储的单位是**当前前沿格子**。每个前沿节点保存其成员格子的引用、世界坐标质心和活跃/休眠状态；节点分数是成员格子融合分数的最大值，节点本身不做加权融合。没有密集语义网格、历史拓扑节点或连线。几何占据、膨胀和 ESDF 网格仍保留，供规划避障使用。

## 处理顺序

1. C++ 用同一时间戳的 depth/sensor_pose 更新地图，立即刷新当前前沿。
2. 发布 `/frontier_semantics/scan`：原始相机位姿、网格坐标参数、本帧射线实际观测到的自由格子，以及当前前沿格子的生命周期 ID 和坐标。
3. 独立 Python 节点精确同步该消息、原始 RGB 和机器人 odom，保存最近 **20 帧**，每帧带原始采集时间戳、相机位姿、机器人位姿及射线支持信息。YOLO/HTTP 推理不阻塞这一收集线程。
4. 当前前沿变化时，遍历窗口中的图像，为每个前沿格子寻找原图中实际观测到的相邻自由格子；结合原始相机方向、视场角和距离判断是否可投影。同一格子可以接受多张图像的证据，绝不只检查质心附近。
5. 只有原图能投影到至少一个尚未接收该图证据的当前前沿格子时，才调用选定后端计算整帧图文匹配分数。其余图像只保留在窗口中等待后续前沿出现。同一张图像只成功推理一次；后续新增前沿复用其分数。
6. 发布原始 Header、backend、score，以及对应格子 ID/投影权重。C++ 对仍存在的每个格子分别融合，然后重新计算各节点的最大值。

终端只输出有前沿格子更新的图像信息，不再输出 `cells=0`。若前沿在模型推理期间消失，
保留计算结果供窗口内后续匹配使用，本次不发布消息或正常评分日志。YOLO 负证据处理只能
复用这些被前沿选中的图像分数，不再为无关图像额外调用语义模型。

离散、连续和手动入口共用此窗口处理；离散/手动入口不会再把检测框画到 `/habitat/camera_rgb` 上。标注图片仍可用于视频或本地显示。

## 可见性、投影与融合

本项目的前沿格子是与自由区域四邻接的未知格子。投影必须有**原图射线实际观测到的四邻接自由格子**作为支持，不能因为某个格子现在是已知区域，就把它当成旧图看见过的区域。位置和光轴方向均来自原图对应的相机位姿；不使用推理完成时的位姿。

对于有效支持格子，使用以下角度/距离权重：

```text
half_fov = 79° / 2
w = cos²(relative_yaw / half_fov × π/2) / (1 + (distance / max_range)²)
V' = (w × S + C × V) / (w + C)
C' = (w² + C²) / (w + C)
node_score = max(V[cell] for scored cells in this frontier)
```

S 是该图像的原始 CLIP cosine / BLIP2 ITC 分数，V/C 是**该格子**的融合值/置信度。
融合公式沿用原设计，增加距离对观测权重的影响；距离分母使用至少一个网格分辨率，避免零值。
超出水平视场、相机后方、零距离或近零权重的格子不更新。

首次有效观测的 V=S、C=w；不是直接把第一次分数改成 w×S。因此一张图像首次覆盖的格子可以同分，但后续会因不同视角和距离得到不同融合结果。这里没有对每个格子单独裁图推理。

同一格子/图像时间戳最多融合一次。由于置信度融合不是交换运算，每格保留最近 20 条证据按采集时间重放，更旧的历史折叠为累计 V/C；早于已折叠部分的迟到结果丢弃。有效负分数也参与最大值，不以零截断。

## 生命周期与内存边界

- 同一个格子持续为前沿时，聚类分裂/合并、质心变化不会清除它的融合记录。
- 格子被扫描后不再为前沿，或所属前沿被规划器移除，立即删除其语义。再次出现时分配新的生命周期 ID，旧结果不会更新它。
- dormant 只是调度状态；仍属于当前边界的休眠格子保留语义，消失时删除。
- 最新图像窗口为 20 帧。一次在途推理批次最多额外持有 20 个旧帧引用；精确同步器使用有界 30 条消息队列。没有无限增长的图像任务队列，也不写图像文件。
- 前沿投影不再依赖旧的 600 帧/15 秒 C++ 语义匹配缓存。该缓存仅保留用于原有目标负证据的降采样深度信息；超时的对象证据仍会跳过。
- 切换目标/回合会清空窗口；在途旧目标推理结果丢弃。现有启动/退出重置服务同时清空地图、前沿及语义，旧时间戳被拒绝。

有限窗口不保证每一帧都完成推理：模型持续慢于采集时，未进入在途批次的旧帧可能被窗口淘汰。刚产生、仍等待推理或窗口内没有有效覆盖图像的格子仍显示未评分；不会用零或邻居的无关分数补齐。节点只要有已评分成员，就显示其最大值。

## 前沿聚类与 RViz

`splitLargeFrontiers(candidate_frontiers_)` 调用已注释，停用按 `frontier.cluster_size_xy=0.65` 对长边界继续做 PCA 拆分。连通区域提取、最小簇过滤及活跃/休眠调度保留。旧参数仍在配置中，但不会再触发此次拆分。

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/frontier_semantics/scan` | `plan_env/FrontierScan` | 本帧原始几何支持与当前前沿格子集合 |
| `/semantic/score` | `plan_env/SemanticScore` | 原图时间戳、模型原始分数、格子 ID 和各自权重 |
| `/frontier_semantics/score` | `sensor_msgs/PointCloud2` | 已评分前沿质心，intensity 为成员格子最大值 |
| `/frontier_semantics/confidence` | `sensor_msgs/PointCloud2` | 质心及最大值格子的融合置信度 |
| `/frontier_semantics/markers` | `visualization_msgs/MarkerArray` | 各前沿格子按各自融合值着色，中心点按簇内最大值着色 |
| `/frontier_semantics/labels` | `visualization_msgs/MarkerArray` | `F<ID> 分数 (已评分格子数/总格子数)`；没有有效成员分数时为 unscored |

RViz 默认开启 **Frontier boundaries and centers**，标签默认关闭。活跃中心是球，休眠中心是方块；语义越高红色越深，未评分为灰色。色阶固定 0.10–0.35，仅显示截断，不改动存储或规划分数；可用 `frontier_visualization/score_min` 和 `score_max` 调整。`F132` 是簇 ID，不是当前前沿数量。每次发布替换完整当前集合，重置删除旧标记。

## 使用和验证

消息接口已经更新，需重新构建并重启规划器、仿真和 RViz。当前容器已编译；运行命令中的 `--mode`、`--semantic-model`、数据集参数不变，模型服务和权重无需重装。

其他工作空间重新编译：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-up-to exploration_manager --symlink-install \
  --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

逻辑测试：

```bash
python -m unittest discover -s tests -v
```

容器内 source ROS/install 并激活 Habitat 环境后，以下测试顺序运行，使用独立 domain 94，不启动 Habitat 场景或模型服务：

```bash
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_publisher.py
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_alignment.py
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_alignment.py --continuous
```

覆盖逐格差异权重、节点最大值、生命周期、重复/乱序证据、20 帧窗口、原始位姿与实际射线支持、模型分数复用、目标切换、ROS 延迟结果及地图/轨迹重置。完整 Habitat 导航与性能评测由实际运行验证。
