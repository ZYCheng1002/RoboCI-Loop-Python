# roboci-loop

机器人硬件、控制、运动学和数据记录的核心 Python 包。

## 安装

在仓库根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ./roboci_loop
```

安装后可从任意工作目录导入：

```python
from roboci_loop.factory import build_arm, build_camera, build_controller
```

## 目录

- `src/roboci_loop/common/`：相机和机器人消息。
- `src/roboci_loop/hardware/`：机械臂、相机、灵巧手和 PICO 适配。
- `src/roboci_loop/controller/`：多设备关节控制。
- `src/roboci_loop/kinematics/`：Placo 正逆运动学。
- `src/roboci_loop/recording/`：机器人与相机数据记录。
- `src/roboci_loop/models/`：随 wheel 安装的 URDF、XML 和网格资源。
- `../configs/`：仓库级应用配置，不随核心包安装。
- `../third_party/`：独立 SDK 和第三方 Python 包。

## 硬件依赖

### Orbbec 相机

使用 `pyorbbecsdk2`。多相机应配置序列号；RGB 输出为 `uint8`，深度单位为米。示例：

```bash
python3 examples/camera_preview.py
```

### RM65 机械臂

使用 RealMan `Robotic_Arm` SDK。控制前确认机械臂 IP、本机 UDP IP、初始姿态和安全速度。

### LG603 灵巧手

SDK 位于 `third_party/linked_hand_python/`，不进入核心 wheel。驱动依次使用：

1. 已安装的 `linked_hand` 模块；
2. `ROBOCI_LINKED_HAND_SDK` 指定的目录；
3. 源码仓库中的 SDK；
4. `HandLG603(..., sdk_path=...)` 显式路径。

SDK 目录中的 `lib/` 和 `vendor/` 必须完整保留。详见 [SDK 说明](../third_party/linked_hand_python/README.md)。

### PICO

使用 `xrobotoolkit-sdk`。运行前启动 XRoboToolkit PC Service 并连接头显应用。

## 主要模块

- `factory`：根据配置创建硬件和 IK 对象，不自动启动或使能设备。
- `RobotController`：为各设备建立控制线程，管理 HOLD、INIT、AUTO 和 TELEOP 模式。
- `RM65IK`：使用随包安装的 RM65-B 模型；位置单位为米，关节单位为弧度，四元数顺序为 xyzw。
- `PoseTeleop`：完成输入位姿到机器人世界坐标系的相对映射和单步 IK。
- `recording`：异步保存机器人消息及相机视频，不负责传感器采样和时间同步。

设备生命周期由应用管理。正常顺序是创建、启动、使能、控制、停止线程、关闭设备。不要让多个进程同时控制同一硬件。

## 常用示例

```bash
python3 examples/lg603_frequency.py
python3 examples/rm65_ik.py
python3 examples/rm65_up_down.py
python3 examples/teleop_pico_example.py
python3 examples/record_teleop.py
```

其中运动和遥操作示例会控制真机；运行前检查工作空间并准备急停。

## 测试

```bash
PYTHONPATH=roboci_loop/src python3 -m unittest discover -s roboci_loop/tests
```
