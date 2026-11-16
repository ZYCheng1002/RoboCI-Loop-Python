# 推理人工纠正与数据采集

该应用组合 OpenPI 推理、PICO 人工接管、MCAP 数据采集、历史轨迹回退和
Web 控制。代码独立放在本目录，不依赖其他 app。

## 状态规则

录制与推理是两套独立控制：

- 无论是否正在推理，都可以开始或结束录制；录制不是启动推理的前提。
- 推理结束后必须由人工提交“成功”或“失败”，是否发生人工接管不会自动决定结果。
- “推理中断”和“结束推理”都会停止本轮并显示“推理成功 / 推理失败”，人工提交后立即更新成功率。
- 接管后可继续遥操作，也可以回退到当前 action chunk 开始前的状态。回退会反向安全重放轨迹；完成后需先松开 Grip，再重新按下接管。
- “回到初始化”可让双臂和灵巧手返回配置的初始位置。

推理提交过的目标按 `action_horizon` 划分控制 chunk。回退时仅反向重放
当前 chunk，并最终到达该 chunk 第一个动作执行前的反馈状态。每个 Task 的累计统计
写入 `<output>/<Task>/stats.json`；未填写 Task 时写入 `<output>/_inference/stats.json`。

## 代码结构

- `control.py`：设备装配、推理状态转换与主控制循环。
- `dashboard_node.py`：ROS Dashboard service、录制、统计和相机发布。
- `side_control.py`：单侧机械臂与灵巧手遥操作。
- `inference_runtime.py`：OpenPI 后台推理线程及辅助函数。
- `web_dashboard.py` / `web_page.py`：HTTP API 与页面模板。
- `record_history.py`：采集历史、元信息和质检文件管理。

## 配置

默认配置为 `configs/human_tele_correct.py`。在
`CONFIG['inference']['prompts']` 中添加网页下拉框可选的 prompt。

## 启动

控制与采集进程：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/human_tele_correct/control.py
```

Web 进程：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/human_tele_correct/web.py
```

访问 <http://127.0.0.1:8088>。两个进程需要使用相同的
`ROS_DOMAIN_ID`、`--prefix` 和 `--config`。
