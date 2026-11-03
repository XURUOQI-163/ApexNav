# 拓扑图与方向关键帧快照

TOPOLOGY 模式自动保存到编译时的项目根目录：

```text
/home/xuyang/nav_repro/ApexNav-ROS2/topology_snapshot/
├── graph.json
└── images/
    ├── node_1/
    │   ├── slot_0_stamp_1793530000000000000.jpg
    │   └── ...
    └── node_2/
```

无需额外下载或启动保存程序。重新编译并重启规划器后，原仿真命令会在启动握手时自动创建目录。此目录已加入 `.gitignore`。

## 生命周期

- 每次启动仿真，原有 `/exploration/reset` **统一清空内存地图、拓扑图和上一轮磁盘快照**。没有独立的磁盘 reset 服务。
- 默认每秒在后台更新一次，最多一个写任务；图像写好后原子替换 `graph.json`，避免正常写入期间出现半截 JSON。
- 正常结束、Ctrl+C 和 SIGTERM 不调用 reset，只发送停车/结束信号，保留内存地图供 RViz 查看，并补存快照。停止后迟到的 RGB 仍可按时间戳补入，后台保存继续运行。
- 仅重启规划器不会清除之前的磁盘数据；下一次仿真启动或显式手动 reset 才会清除。
- 强制 `kill -9`、进程崩溃时保留最近完成的快照，不能保证最后一秒内的新数据；不保证断电后的文件系统持久性。
- 保存的是**最新图状态**，不是每一帧的历史录像。图中删除的节点及被替换的图片会在新 JSON 发布后从目录清理。需要保留多轮结果时，在启动下一轮前自行复制整个目录。

`snapshot.interval`（秒）位于 `config/topology.yaml`，默认 1.0。可通过节点参数 `snapshot.directory` 或规划器进程的 `APEXNAV_TOPOLOGY_SNAPSHOT_DIR` 环境变量指定目录；目录名必须为 `topology_snapshot`。测试使用临时目录，不覆盖项目中的用户快照。

## graph.json 字段

所有位置以 `world` 为参考系，米为距离单位，弧度为角度单位，`*_ns` 为纳秒整数时间戳。无法确定的距离写为 `null`，并由状态区分 `UNOBSERVED` / `INVALID` / `FREE` / `OBSTACLE`。

| 数据 | 字段 |
|---|---|
| 整体 | `schema_version`、`frame_id`、`started_at_ns`、`captured_at_ns`、`planner_state`、当前 WAYPOINT 和节点/边数量 |
| 节点 | `id`、`type`、`position_xy_m`、`source_id`、`measured_at_node`、`stamp_ns` |
| 描述子 | `origin_xy_m`、时间戳、最大距离、角宽、`sectors`（默认 12 个）、`directions`（默认 72 个） |
| 每个方向 | 索引、实际中心朝向 `center_yaw_rad`、状态、自由观测范围、最近障碍距离、时间戳、有效/无效采样数；粗扇区另有覆盖 mask |
| 图片 | 30° 方向槽 `slot`、精确采集时间戳、相机 `camera_position_xyz_m`、`yaw_rad`、四元数 `orientation_xyzw`、相对文件 `path`、文件大小 |
| 边 | 两端 ID `a/b`、`weight_m`、`verified`、`verified_at_ns`、`traversed`、完整 `path_xy_m` 折线 |

`captured_at_ns` 是本次导出时间；图片和各方向有自己的观测时间。判断图像与节点对应关系应查看观测时间和实际相机位姿。

FRONTIER 尚未到达，其描述子与图像来自来源 WAYPOINT；检查 `source_id`、`measured_at_node=false`、描述子原点及相机位姿，不能把该图误当作前沿位置拍摄。到达并转换为 WAYPOINT 后保存当地观测。每个节点最多 12 个方向图片槽，JPEG 保持现有 320 像素宽、质量 75；未匹配到同时间戳 RGB 时不生成假图片，`pending_image_count` 表示等待匹配数量。

在宿主机用文件管理器打开 `images/node_<id>/` 即可看图。检查 JSON：

```bash
cd /home/xuyang/nav_repro/ApexNav-ROS2
python -m json.tool topology_snapshot/graph.json | less
```

完整节点与边结构来自 C++ 图本身，未通过点云或 RViz Marker 反推。
