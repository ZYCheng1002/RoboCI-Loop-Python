# 推理人工介入

使用 OpenPI 控制双臂，并持续监听 PICO 手柄。任一侧 Grip 超过配置阈值后，系统立即永久结束本次自动推理，切换至与 `pico_rm65_teleop` 相同的双臂和灵巧手遥操作。松开 Grip、再次按下或执行其他手柄操作都不会恢复推理。

当前应用不包含界面、录制和 ROS2 服务。

## 启动

先确认 OpenPI 服务、PICO PC Service 及机器人硬件已经就绪：

```bash
source .venv/bin/activate
python3 apps/inference_review/inference_review.py \
  --config configs/inference_review_config.py
```

默认读取 `configs/inference_review_config.py`。OpenPI 服务、prompt、broker、
推理频率、观测超时和是否回初始姿态均在其 `inference` 段配置。

## 接管与手柄操作

- 任一 Grip：首次按住时永久停止推理；之后按住对应机械臂跟随，松开后保持。
- 左 Y / 右 B：对应机械臂回到初始姿态。
- 左 X / 右 A：对应灵巧手进入 First 抓取姿态。
- Trigger：控制食指到小指闭合。
- 摇杆上下：控制拇指屈曲。
- 摇杆左右：控制拇指旋转，左右手方向镜像。

Ctrl+C 退出整个应用。
