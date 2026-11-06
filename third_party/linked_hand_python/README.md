# linked_hand_python

LG603 灵巧手的 Linux x86_64 Python 运行包，使用 `ctypes` 调用随目录提供的动态库。

## 目录要求

以下内容需要一起保留：

- `linked_hand.py`：Python 接口。
- `lib/`：灵巧手 SDK 动态库。
- `vendor/`：GCAN 和 USB 依赖。
- `udev/`：可选 USB 权限规则。

运行环境需要 Python 3.8+，以及兼容的 glibc、libstdc++ 和 USB 访问权限。

## 在 RoboCI 中使用

源码仓库会自动找到该目录。独立部署时可设置：

```bash
export ROBOCI_LINKED_HAND_SDK=/绝对路径/linked_hand_python
```

也可以将目录加入 `PYTHONPATH` 后直接导入 `linked_hand`。

## 频率测试

只读取双手：

```bash
python3 third_party/linked_hand_python/examples/benchmark.py --seconds 30
```

同时读取和控制：

```bash
python3 third_party/linked_hand_python/examples/benchmark.py --control --seconds 30
```

默认使用同一个 GCAN 设备：左手 channel 0、ID `0x65`，右手 channel 1、ID `0x64`，波特率 1 Mbps。测试发送启动时读取到的位置，不会自动使能、校准或归零。

## 注意事项

- Python 接口角度单位为度；RoboCI 的统一接口使用弧度。
- 六个关节逐个通信，不是原子采样或原子控制。
- 关闭设备前必须结束所有读写线程。
- 不要让 C++ 和 Python 程序同时占用同一个 USB 适配器。

版本与校验信息见 `MANIFEST.json`，第三方声明见 `THIRD_PARTY_NOTICES.md`。
