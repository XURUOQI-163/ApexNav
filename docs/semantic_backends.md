# CLIP / BLIP2 选择与连续模式时间对齐

本次保持 `vlm/similarity/clip.py` 和 `vlm/utils/get_clip_similarity.py` 的模型、提示词、HTTP 接口不变。
BLIP2 独立位于 `vlm/similarity/blip2.py`、`vlm/utils/get_blip2_similarity.py`。
BLIP2 实现参考本地 Git HEAD `1ec9d155fc972fb8a139bd57f43b65247237601e` 中原版
`vlm/itm/blip2itm.py`，使用 LAVIS `blip2_image_text_matching/pretrain` 的 **ITC** 头。
不把 ITM softmax 当作余弦相似度，不同时计算无用的 ITM 输出。

## 模型服务（二选一）

YOLOE 继续按原命令运行。CLIP 与 BLIP2 默认端口不同，代码、模型依赖相互独立。
推理客户端只使用 HTTP，不要求 Habitat 环境安装 LAVIS 或 torch。

CLIP（原容器内命令）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/apexnav_vision/bin/activate
python -m vlm.similarity.clip --port 12182 \
  --model-name ViT-B/32 --download-root data/clip
```

BLIP2（宿主机，使用已下载好的 Conda 环境与缓存）：

```bash
conda activate apexnav_blip2
cd /home/xuyang/nav_repro/ApexNav-ROS2
export TORCH_HOME="$PWD/data/blip2/torch"
export HF_HOME="$PWD/data/blip2/huggingface"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
python -m vlm.similarity.blip2 --port 12185 --device cuda --model-type pretrain
```

容器 `apexnav-ros2-jazzy` 使用 host 网络，可以通过 127.0.0.1 访问宿主机服务。
不用同时加载两个模型进行导航；选择 BLIP2 时可关闭 CLIP 以释放显存。
此处 BLIP2 离线模式使用用户已准备的权重，服务加载不会安装依赖。

| 后端 | HTTP | 默认端口 | 响应 |
|---|---|---|---|
| clip | POST /clip | 12182 | cosine |
| blip2 | POST /blip2 | 12185 | cosine、backend、head、timing |

可在仿真终端用 `CLIP_SERVER_URL` / `BLIP2_SERVER_URL` 覆盖完整 HTTP 地址；启动仿真前设置。
两个服务都支持 `GET /health`。BLIP2 端口示例：`curl http://127.0.0.1:12185/health`。

## 规划器和仿真

ROS 消息已变化，需重新编译依赖包并重启旧节点。本次已在现有 Jazzy 容器完成构建；
如果使用其他工作空间或容器，执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-up-to exploration_manager --symlink-install --executor sequential \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

规划器终端：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager simulation.launch.py mode:=continuous
```

仿真终端（容器内）：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
export ROS_DOMAIN_ID=31
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
unset LIBGL_ALWAYS_SOFTWARE
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

python habitat_evaluation.py --mode continuous --semantic-model blip2 \
  --dataset hm3dv2 test_epi_num=150 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets
```

CLIP 将 `--semantic-model blip2` 改为 `--semantic-model clip`；省略该参数也使用 CLIP。
离散模式同样支持 `--semantic-model`，同时将仿真和规划器的模式改成 discrete。
切换模型前结束旧仿真并重启规划器，避免不同模型的历史分数混合。规划器参数无需指定模型。
原 `habitat_manual_control.py` 继续使用 CLIP，消息已更新为带时间戳版本。

## 时间对齐设计

RGB、depth、sensor_pose、robot odom 共用采集时间戳。C++ 在每次建图后发布
`/frontier_semantics/scan`，携带原图几何支持和当前前沿格子。独立收集线程精确同步
RGB/scan/odom，保留最新 20 帧；模型推理使用原图、原相机/机器人位姿和缓存分数。

`/semantic/score`（`plan_env/msg/SemanticScore`）包含原始 Header、backend、score，
以及当前格子 ID/投影权重。先逐格融合，再取成员格子的最大值作为前沿节点分数。
重复证据只融合一次，删除的格子、重置前的结果和混合后端结果拒绝写入。
同一格子最近 20 条证据按采集时间重放，更早历史折叠保存，不随整个场景无限增长。

仅在原图覆盖当前前沿格子且存在尚未发送的证据时，才推理或复用该图像分数，并输出日志。
无匹配前沿的图像只缓存在窗口中；前沿在推理期间消失时，本次不发布空更新或评分日志。
YOLOE 未检出物体的历史置信度更新只能复用这些已选图像的原图分数和深度，不再触发额外推理。
这个对象证据缓存保留默认 600 帧/15 秒限制，参数为
`semantic/cache_size`、`semantic/max_observation_age`。它不再限制前沿格子的回溯融合。

终端 `raw` 是模型原始分数，`cells` 是该条消息的投影格子数量，`cached` 表示是否复用了
图像分数，不是节点融合值。融合结果查看 RViz 或 `/frontier_semantics/score`。
CLIP 和 BLIP2 的 HTTP 协议、模型、环境、端口独立；旧 `/clip/cosine_score` 无时间戳话题
不参与建图。详细公式、内存边界和有限窗口的限制见 [前沿逐格语义记忆](frontier_semantics.md)。

BLIP2 与 CLIP 得分尺度不同，不承诺换模型即可提高成功率。

## 验证

无 ROS 的完整测试（环境需有项目既有测试依赖，C++ 缓存测试需 g++）：

```bash
python -m unittest discover -s tests -v
```

容器内 source ROS/install 并激活 Habitat 环境后：

```bash
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_alignment.py
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_publisher.py
```

集成测试使用独立 ROS_DOMAIN_ID=94。第一个启动真实 C++ 地图节点：A 帧之后相机移至
8 米外的 B，再发送延迟 A 分数和重复分数，验证只在 A 融合一次，再验证 B。
第二个使用受控推理延迟，验证两种后端的独立收集、20 帧窗口、精确时间戳、缓存复用、
目标切换、无关图像不调用模型、前沿出现后选择旧图，以及在途前沿消失时跳过空更新。
此次没有运行真实模型或下载资源。
这不是完整导航成功率评测。
