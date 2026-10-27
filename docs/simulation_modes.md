# 仿真模式选择

> 更新：现在支持独立的 CLIP / BLIP2 后端及带时间戳的语义消息。当前启动方式和协议见 [语义后端与时间对齐](semantic_backends.md)。

在统一入口 `habitat_evaluation.py` 中使用 `--mode discrete` 或 `--mode continuous`。
省略 `--mode` 时仍使用离散模式。`--dataset` 和已有 Hydra 路径、任务编号参数保持可用。
这两个模式需要各自对应的规划器；切换前停止旧仿真和旧规划器，YOLOE/CLIP 服务可以保持运行。
不要同时运行旧的 `real_world_test_habitat.py`，连续入口已经自动管理这个视觉处理节点。

## 安装新增的 ROS2 启动文件

增加了 Python 启动入口，并修复连续控制器首次收到轨迹前处理停止消息时的时钟类型异常。
容器 `apexnav-ros2-jazzy` 中已完成以下构建；以后换容器或重新构建工作空间时执行：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select trajectory_manager exploration_manager --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

若控制器已构建、只是尚未安装新增 launch 文件，可以直接从源码路径启动：

```bash
ros2 launch /home/xuyang/nav_repro/ApexNav-ROS2/src/planner/exploration_manager/launch/simulation.launch.py mode:=continuous
```

## 规划器终端

先 source ROS2 和工作空间，并与仿真保持相同的 `ROS_DOMAIN_ID`：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=31
```

二选一：

```bash
ros2 launch exploration_manager simulation.launch.py mode:=discrete
```

```bash
ros2 launch exploration_manager simulation.launch.py mode:=continuous
```

离散模式启动原来的动作规划器；连续模式启动轨迹规划器和 MPC 轨迹执行节点，输出 `/cmd_vel`。
上游内部的 `is_real_world=true` 用于选择轨迹状态机；新启动入口的两种模式均连接 Habitat 话题，
不启动实车驱动。

## 仿真终端

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
```

离散模式：

```bash
python habitat_evaluation.py --mode discrete --dataset hm3dv2 test_epi_num=150 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets
```

连续模式：

```bash
python habitat_evaluation.py --mode continuous --dataset hm3dv2 test_epi_num=150 \
  llm.llm_client.llm_client=none \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets
```

连续入口会自动启动一个独立视觉处理进程，用同一数据集配置调用已有 YOLOE 和 CLIP 服务；
推理不阻塞运动循环。退出仿真时这个子进程也会结束。不要另外重复启动它。
如果 10 秒内没有发现 `/cmd_vel` 发布者，入口会提示切换到连续规划器，而不会直接开始转动。

## 行为和参数

| 项目 | 离散 | 连续 |
|---|---|---|
| 执行接口 | `/habitat/plan_action` 动作编号 | `/cmd_vel` 的线速度与角速度 |
| 位姿更新 | 前进 0.25 m，转向 30° | 按速度和时间步积分，再做 NavMesh 约束 |
| 默认预算 | 原配置的 500 个动作步 | 9000 个控制周期，30 Hz 下约 300 秒仿真时间 |
| 视觉处理 | 每个动作后同步调用 | 独立进程处理图像、深度和位姿 |
| 任务运行方式 | 沿用原评测循环 | 指定一个 episode，结束后打印指标并退出 |
| 可视化 | 原离散 RViz 配置 | 推荐 `rviz_traj.launch.py` |

连续模式参数可直接加在命令末尾：

```text
continuous.control_hz=30.0
continuous.command_timeout=0.5
continuous.initial_scan_seconds=4.0
habitat.environment.max_episode_steps=9000
```

- `control_hz`：速度积分频率；计算慢时实际墙钟频率可能低于设定值。
- `command_timeout`：超过该秒数没有收到新速度指令，速度归零。
- `initial_scan_seconds`：开始阶段以 90°/秒原地扫描，完成后自动触发规划；设为 0 可取消扫描。
- `max_episode_steps`：连续模式计的是控制周期，不是离散导航动作。修改频率时需同时考虑时间预算。
- 连续模式强制要求 `forward_step_size=0`，因为位移已由速度积分完成；随后的零位移动作只更新传感器和任务计数。
- 连续模式的 `test_epi_num` 是 `env.episodes` 列表的零基索引，会打印实际 `episode_id` 和场景；默认 `-1` 选择第一个任务，不批量运行。
- 连续模式当前提供实时图像/RViz 和结束指标，不写离散评测的汇总记录；`need_video=true` 会明确报错，避免静默忽略。
- 两种模式的步数/时间预算不同，不能直接将结果当成同一评测设置。
- 连续模式使用速度积分和 NavMesh 碰撞约束，不是包含轮胎、惯性和接触力的实车动力学仿真。
- Habitat 内置碰撞计数读取的是随后执行的零位移动作，不代表速度积分时被 NavMesh 阻挡的次数。
- 本次没有修改之前提到的 RViz 标记清理问题，也没有重写离散模式的任务循环。

连续模式 RViz：

```bash
ros2 launch exploration_manager rviz_traj.launch.py
```

模式及速度超时测试：

```bash
python -m unittest discover -s tests -v
```

本次验证：24 项自动测试通过；两个 ROS2 包构建通过。在独立 ROS domain 的 HM3Dv2
第 150 个任务上运行 600 个连续控制周期，收到 600 条里程计，最高实测线速度约
0.49 m/s，终点相对起点位移约 3.16 m。此短时验证用于确认完整运动链路，不代表导航成功率评测。
