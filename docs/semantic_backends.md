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

1. 同一帧 RGB、深度和 sensor_pose 共享采集时间戳；连续视觉端使用精确时间同步。
2. C++ 建图仍持续处理深度。每帧缓存当时的相机位姿、可见自由格子和紧凑深度点云。
3. 推理完成后发布 `/semantic/score`（`plan_env/msg/SemanticScore`）：原始 Header、backend、score。
4. C++ 只用时间戳完全相同的缓存观测融合语义，不使用推理完成时的位置。
   分数先到、深度先到均支持；重复分数只融合一次。旧于已融合结果的乱序分数丢弃。
5. YOLOE 点云消息增加原始 Header。检测过滤和未检出物体的历史置信度更新也使用对应帧；
   缺少对应深度、检测或语义结果时跳过该条证据，不拿其他帧补齐。
6. 地图重新初始化时清空缓存，并拒绝初始化之前的观测。

旧 `/clip/cosine_score` 无时间戳话题不再参与建图；CLIP 自身 HTTP 协议不变。
两种模型的权重、模型代码、环境、服务端口独立，共用带时间戳的 ROS 输出协议。
语义融合公式、YOLOE 模型、探索阈值和 RViz 标记清理均未在本次调整。

缓存默认最多 600 帧、15 秒（以先到的限制为准）。规划器 ROS 参数为
`semantic/cache_size` 和 `semantic/max_observation_age`，在节点初始化时读取。
超过限制、消息丢失或深度/位姿时间戳不一致时丢弃对应语义结果，不能保证每帧都融合。
运行时会打印原始分数、采集时间戳、观测年龄；规划器 `[Semantic]` 输出缓存和丢弃计数。
BLIP2 得分尺度与 CLIP 不同，首轮对照保持相同提示词/阈值；不承诺换模型即可提高成功率。

## 验证

无 ROS 的完整测试（环境需有项目既有测试依赖，C++ 缓存测试需 g++）：

```bash
python -m unittest discover -s tests -v
```

容器内 source ROS/install 并激活 Habitat 环境后：

```bash
python tests/ros2_semantic_alignment.py
PYTHONPATH="$PWD:$PYTHONPATH" python tests/ros2_semantic_publisher.py
```

集成测试使用独立 ROS_DOMAIN_ID=94。第一个启动真实 C++ 地图节点：A 帧之后相机移至
8 米外的 B，再发送延迟 A 分数和重复分数，验证只在 A 融合一次，再验证 B。
第二个使用受控推理延迟，验证两种后端的 Python 发布端保留原帧时间戳。
另已使用本机缓存 BLIP2 权重完成实际 CUDA ITC 推理及 Flask HTTP 请求检查；未下载资源。
这不是完整导航成功率评测。
