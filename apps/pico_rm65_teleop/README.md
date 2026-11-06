# PICO 双臂遥操作

通过 PICO 控制双臂 RM65 和 LG603 灵巧手，并使用 Web 页面预览相机、录制数据和质检。

## 准备

1. 安装项目依赖、ROS2 Jazzy 和 PICO SDK。
2. 修改 `configs/record_pico.py` 中的设备 IP、CAN 通道、相机序列号、初始姿态和记录参数。
3. 确认机器人工作空间安全并准备急停。

## 启动

遥操作终端：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/pico_rm65_teleop/teleop.py
```

Web 终端：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/pico_rm65_teleop/web.py
```

访问 <http://127.0.0.1:8088>。远程访问可为 Web 增加 `--host 0.0.0.0`。两个进程必须使用相同的 `ROS_DOMAIN_ID`、`--prefix` 和 `--config`。

## 手柄操作

- Grip：按住跟随机械臂，松开后保持当前位置。
- 左 Y / 右 B：对应机械臂回到初始姿态。
- 左 X / 右 A：对应灵巧手进入 First 抓取姿态。
- Trigger：基于当前姿态控制食指到小指闭合。
- 摇杆上下：控制拇指屈曲。
- 摇杆左右：控制拇指旋转，左右手方向镜像。

## 数据录制

在页面填写 Task、操作员、记录员和点位后开始录制。默认目录为：

```text
~/datasets/<Task>/<时间戳>/
```

每次记录包含 MCAP、rosbag2 元数据、配置快照和质检结果。可通过 `teleop.py --output-dir` 修改根目录。选择“废弃”会将记录移动到对应 Task 的 .trash/ 目录；历史页面的删除操作会永久移除正常记录。

所有实际收到的相机帧都会录制；页面预览频率和尺寸不会改变原始采集频率。JPEG 质量由 `recording.rgb_jpeg_quality` 配置。

## 测试

无硬件集成测试：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/pico_rm65_teleop/test_integration.py
```

Ctrl+C 退出时，先停止控制和采集线程，再等待已接收数据写入完成。
