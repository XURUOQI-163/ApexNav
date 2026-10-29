# 云端千问目标先验

旧实现先读取 `llm/answers/llm_answer_hm3d.txt`（MP3D 使用另一文件）。即使选择
`llm.llm_client.llm_client=none`，命中缓存仍使用其内容；未命中则返回
`everywhere`、0.5。`ollama` 指本地模型，和云端 API 是不同入口。

云端入口是 `llm/client/qwen_answer.py`，通过已有 OpenAI SDK 调用兼容接口。
只发送目标文本，返回房间先验和融合阈值。YOLOE 仅检测原目标（及其别名）；
房间与原目标共同组成 BLIP/CLIP 提示；阈值沿原接口发送至规划器。
原目标不会被千问改写。房间是常识先验，不能保证当前场景真的存在该房间。

在运行仿真的容器终端中激活现有环境后设置密钥（输入不回显，不写进命令历史）：

```bash
read -rsp 'DashScope API Key: ' DASHSCOPE_API_KEY; echo
export DASHSCOPE_API_KEY
export DASHSCOPE_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
export QWEN_MODEL=qwen-plus
python -m llm.check_qwen 'yellow chair'
```

`check_qwen` 会真实调用一次云端，只打印解析后的先验，不写缓存。密钥、模型和
服务地址应与百炼控制台对应地域/工作空间匹配；第三方兼容网关需确认支持
`response_format=json_object` 与 `enable_thinking=false`。不需要下载模型。
环境变量仅影响当前终端及其子进程；新开的 `docker exec` 不会自动继承。
配置中也可设置 `llm.llm_client.qwen.model/base_url/timeout`，但前两个环境变量优先。
API Key 仅从 `DASHSCOPE_API_KEY` 读取，不写入 YAML 或缓存。

仿真命令示例（其余 ROS 环境沿用原启动设置）：

```bash
python habitat_evaluation.py --mode continuous --semantic-model blip2 \
  --target 'yellow chair' --dataset hm3dv2 test_epi_num=550 \
  llm.llm_client.llm_client=qwen \
  llm.llm_client.cache_policy=read_write \
  habitat.dataset.data_path=/home/xuyang/nav_repro/ApexNav-Lite/data/datasets/objectnav/hm3d/v2/val/val.json.gz \
  habitat.dataset.scenes_dir=/home/xuyang/nav_repro/ApexNav-Lite/data/scene_datasets \
  habitat.simulator.habitat_sim_v0.allow_sliding=true
```

离散模式也使用相同参数。连续模式的感知子进程继承这些配置与环境变量。
仅在读取新目标上下文时请求 LLM，不对每张图片调用千问。

缓存策略：

- `read_write`：默认；优先读取该千问配置的缓存，未命中才请求并保存。
- `refresh`：不读取旧答案，每次目标上下文初始化都重新请求并追加，后续读取最新条目。
- `off`：不读、不写缓存，仍请求云端。

千问缓存自动生成在原缓存文件旁，例如
`llm/answers/llm_answer_hm3d.qwen-<hash>.txt`；哈希来自模型、API 地址和提示词，
因此不会读到预置的旧答案，也不会混用其他千问模型/提示词版本的答案。
原始回复另存带相同后缀的 response 文件。终端显示 `requesting context`、
`cache hit`、`cache saved` 及实际缓存路径。

请求异常不缓存；离散入口会报错退出。连续入口沿用已有容错：记录
`Failed to read target context`，该目标退回`everywhere`/0.5；
修好网络或配置后需重新启动任务才能再次尝试同一目标。不能把这种情况当成千问成功。

代码检查使用模拟 API，无需密钥。实际 API 连通性和模型权限请用上面的检查命令验证。

已移除相似类别推理与检测，预置缓存现在只保存 `[阈值, 房间]`。
千问因提示词变化自动启用新缓存，不读取旧版本回答。
已有自定义缓存若仍为旧格式会提示并跳过；在线后端重新请求，`none` 使用默认先验。
