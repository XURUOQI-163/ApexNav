# ROS2 仿真：YOLOE + CLIP

> 更新：现在支持独立的 CLIP / BLIP2 后端及带时间戳的语义消息。当前启动方式和协议见 [语义后端与时间对齐](semantic_backends.md)。

此工程已将视觉链路替换为 ApexNav-Lite 使用的 YOLOE 分割模型和 CLIP ViT-B/32。
仅保留两个模型服务，不需要旧的检测器、独立分割服务或 LAVIS。
下列安装、下载、编译和启动命令需要你自行执行；代码适配过程未执行这些操作。

## 接口与工作流

| 功能 | HTTP 服务 | ROS2 输出 |
|---|---|---|
| YOLOE 文本提示目标检测、实例分割 | `POST http://127.0.0.1:12184/yoloe` | `/yoloe/clouds_with_scores`，`plan_env/msg/MultipleMasksWithConfidence` |
| CLIP 场景语义相似度 | `POST http://127.0.0.1:12182/clip` | `/semantic/score`，`plan_env/msg/SemanticScore`（含采集时间戳） |

Habitat RGB → 两个 HTTP 服务 → Python 把 YOLOE 掩码与 Habitat 深度、位姿转换为点云 → ROS2 发布 → C++ 地图与规划。
CLIP 输出原始余弦相似度 [-1, 1]，不生成匹配概率。地图中的负分处理沿用原逻辑。
YOLOE 的 boxes 为归一化 XYXY，masks 必须与原始 RGB 高宽相同；没有实例时返回空列表。
每个点云、分数与类别索引一一对应，目标及目标别名为 0，相关物体为正整数。
`/detector/confidence_threshold` 仍是规划器的物体融合阈值，和 YOLOE 的检测阈值是不同参数。

旧模型的 Python 模块、配置键与旧话题已移除。CLIP 使用新的 `/clip` 接口与 `cosine` 字段，
不与 Lite 原 `/clipitm` 的 HTTP 响应兼容；复用的是模型、权重和编码方式，请运行本工程提供的服务代码。
YOLOE 保留 Lite 的 `/yoloe` 请求结构，但本适配要求真实掩码，并启用 `retina_masks=True` 保持原图坐标。
RGB 在服务端进入 Ultralytics 前转为 BGR；CLIP 接收 RGB。

## 1. 仿真环境与服务环境

继续使用现有 Jazzy 容器 `apexnav-ros2-jazzy` 和 `/opt/apexnav_ros2_py312` 仿真环境。
ROS2 `rclpy` 使用 Jazzy 系统 Python 3.12。Habitat 与这个解释器的兼容问题需先单独解决；
本次改动没有修改或重装 Habitat。

建议新建仅用于视觉服务的环境，避免影响已安装的 Habitat。以下在 **Jazzy 容器内** 执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
/usr/bin/python3 -m venv /opt/apexnav_vision
source /opt/apexnav_vision/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121
python -m pip install -r requirements-vision.txt
python -m pip check
```

上述 CUDA 12.1 wheel 需容器可访问 NVIDIA GPU、宿主驱动支持；可用
`python -c "import torch; print(torch.__version__, torch.cuda.is_available())"` 检查。
若已有能运行 Lite 模型的环境，可直接使用它运行本 ROS2 工程下的服务模块，无须重复创建环境。
服务环境不用 source ROS，也不安装 Habitat。新版客户端不导入 torch、Ultralytics、CLIP 或 Flask。
不要再使用原来的 Python 3.9 Conda 锁文件安装 Jazzy 仿真环境；该锁文件已移除。

可选：现有仿真环境已经具备项目依赖时，以 editable 模式登记代码：

```bash
source /opt/ros/jazzy/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
cd /home/xuyang/nav_repro/ApexNav-ROS2
python -m pip install --no-deps -e .
```

## 2. 权重：优先复用 Lite

现有 Lite 权重路径可直接传给服务，无须复制：

```bash
ls -lh /home/xuyang/nav_repro/ApexNav-Lite/data/yoloe-11l-seg.pt
```

YOLOE 8.3.209 的文本提示编码还可能需要 `mobileclip_blt.ts`；它是 YOLOE 内部文本编码器，
不是第三个 HTTP 服务。启动时在工程根目录运行，若 Lite 根目录已有此文件，可复制到 ROS2 根目录。
若权重只在旧容器内，可在宿主机用 `docker cp` 导出到挂载目录，再在新容器中使用。

CLIP 默认缓存为服务进程用户的 `~/.cache/clip/ViT-B-32.pt`。
旧容器的 `/root/.cache/clip` 不会自动出现在新容器，可以在 **宿主机** 执行：

```bash
mkdir -p /home/xuyang/nav_repro/ApexNav-ROS2/data/clip
# 先确认旧容器确实有这个文件，再复制
# docker exec apexnav-lite-noetic ls -lh /root/.cache/clip/ViT-B-32.pt
docker cp apexnav-lite-noetic:/root/.cache/clip/ViT-B-32.pt \
  /home/xuyang/nav_repro/ApexNav-ROS2/data/clip/ViT-B-32.pt
```

若没有缓存，在视觉服务环境中自行下载：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
mkdir -p data/clip
curl -fL --retry 3 \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yoloe-11l-seg.pt \
  -o data/yoloe-11l-seg.pt
curl -fL --retry 3 \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/mobileclip_blt.ts \
  -o mobileclip_blt.ts
python -c "import clip; clip.load('ViT-B/32', device='cpu', download_root='data/clip')"
```

权重下载地址以 [Ultralytics assets](https://github.com/ultralytics/assets/releases/tag/v8.3.0) 为准；
CLIP 缓存/下载行为见 [CLIP 源码](https://github.com/openai/CLIP/blob/main/clip/clip.py)。
下载时使用你已验证可用的代理，`no_proxy`/`NO_PROXY` 包含 `localhost,127.0.0.1,::1`，防止本地模型请求进入代理。

## 3. 重新编译 ROS2

C++ 订阅话题已变更，必须重新编译并重启规划器。用现有构建环境，在容器内执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 4. 启动两个视觉服务

各开一个容器终端，在两个终端都执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/apexnav_vision/bin/activate
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
```

终端 A（直接复用 Lite 权重）：

```bash
python -m vlm.detector.yoloe --port 12184 \
  --weights /home/xuyang/nav_repro/ApexNav-Lite/data/yoloe-11l-seg.pt
```

若下载到 ROS2 的 `data/`，将 `--weights` 改为 `data/yoloe-11l-seg.pt`。
终端 B：

```bash
python -m vlm.similarity.clip --port 12182 --model-name ViT-B/32 \
  --download-root /home/xuyang/nav_repro/ApexNav-ROS2/data/clip
```

也可用 `--model-name /绝对路径/ViT-B-32.pt` 直接加载本地文件。
两个服务都支持 `--device cpu`；YOLOE 的 GPU 参数示例为 `--device cuda:0`。
服务默认仅监听本机；当前 Docker `--network host` 模式可以直接访问。
同一宿主机运行旧 Lite 服务会占用相同端口，先停止旧进程，或用新端口启动并设置客户端完整 URL：

```bash
# 仅在改端口时，在每个仿真/测试终端设置
export YOLOE_SERVER_URL=http://127.0.0.1:12284/yoloe
export CLIP_SERVER_URL=http://127.0.0.1:12282/clip
```

## 5. 检查并运行仿真

现在支持 `habitat_evaluation.py --mode discrete|continuous`。下面是默认离散模式；
连续模式的匹配规划器、完整命令和预算说明见 [仿真模式选择](simulation_modes.md)。

先用任一具备 numpy、OpenCV、requests 的 Python 环境检查服务：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
python -m vlm.check_services
# 也可使用真实图片：python -m vlm.check_services --image /path/to/room.jpg --label chair
```

空白测试图没有检测结果是正常情况。运行自动化接口测试（模型适配测试需要 torch，但不加载权重）：

```bash
python -m unittest discover -s tests -v
```

规划器终端：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch exploration_manager exploration.launch.py
```

仿真终端：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source /opt/apexnav_ros2_py312/bin/activate
export ROS_DOMAIN_ID=31
export no_proxy=localhost,127.0.0.1,::1
export NO_PROXY="$no_proxy"
python habitat_evaluation.py --dataset hm3dv2 test_epi_num=10 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets
```

`test_epi_num=10` 是指定 episode 编号。RViz2 可在另一个 source 同样 ROS2 环境且 `ROS_DOMAIN_ID=31` 的终端启动：
`ros2 launch exploration_manager rviz.launch.py`。

运行后检查 `ros2 topic info /clip/cosine_score` 和 `ros2 topic info /yoloe/clouds_with_scores`，
应看到对应发布者和规划器订阅者。`ros2 topic echo /clip/cosine_score --once` 可查看分数。

## 参数与验证边界

`config/habitat_eval_*.yaml` 使用 `detector.yoloe.confidence_threshold`、`iou_threshold`、`agnostic_nms`。
评测 episode 上限和步长沿用 ROS2 工程。YOLOE/CLIP 的数值分布不同于原模型，
LLM 答案缓存中的融合阈值尚未针对新模型标定；正式对比导航指标前应重新评测。
接口测试覆盖序列化、掩码尺寸、类别对齐、空检测、负余弦分数和服务错误处理，
不等价于 Jazzy C++ 构建或真实 GPU/Habitat 端到端验证。
