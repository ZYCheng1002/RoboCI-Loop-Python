# RoboCI-Loop-Python

面向灵巧机器人操作的数据采集、遥操作、推理与人工校正项目。名称中的 CI 表示 Collect + Infer。

## 主要内容

- `roboci_loop/`：可安装的核心 Python 包。
- `apps/`：可直接运行的采集和遥操作应用。
- `configs/`：设备与应用配置。
- `examples/`：硬件、运动学和记录示例。
- `third_party/`：独立第三方 SDK，不随核心包自动安装。

## 安装

建议在仓库根目录创建虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ./roboci_loop
```

OpenPI 客户端按需单独安装：

```bash
python -m pip install -e ./third_party/openpi-client
```

部分硬件还需要厂商 SDK、系统动态库和 USB/CAN 权限，详见 [核心包说明](roboci_loop/README.md)。

## PICO 双臂遥操作

先修改 [configs/record_pico.py](configs/record_pico.py)，然后在两个终端分别运行：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/pico_rm65_teleop/teleop.py
```

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python3 apps/pico_rm65_teleop/web.py
```

浏览器访问 <http://127.0.0.1:8088>。操作方式和数据目录见 [应用说明](apps/pico_rm65_teleop/README.md)。

> 运行真机前请核对 IP、CAN 通道、相机序列号、初始关节角和工作空间，并准备急停。
