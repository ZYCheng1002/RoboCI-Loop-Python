"""显式选择实现；只创建对象，不启动设备，也不执行运动。

设备 spec 格式：{"type": "rm65", "config": {"ip": "192.168.1.18"}}。
config 同时接受公共 Config 字段和具体驱动的关键字参数。
"""
from collections.abc import Mapping
from dataclasses import fields
from typing import TYPE_CHECKING, Any
from loguru import logger

if TYPE_CHECKING:
    from .hardware.arm import ArmBase
    from .hardware.camera import CameraBase
    from .hardware.controller import ControllerBase
    from .hardware.end_effector import EndEffectorBase
    from .kinematics import PlacoIK


def _parse(spec: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    if not isinstance(spec, Mapping):
        raise TypeError("设备配置必须是包含 type 和 config 的映射")
    unknown = set(spec) - {"type", "config"}
    if unknown:
        raise ValueError(f"未知配置字段：{sorted(unknown, key=str)}")
    kind = spec.get("type")
    if not isinstance(kind, str) or not kind.strip():
        raise ValueError("type 必须为非空字符串")
    params = spec.get("config", {})
    if not isinstance(params, Mapping) or any(not isinstance(k, str) for k in params):
        raise TypeError("config 必须是键为字符串的映射")
    return kind, dict(params)


def _device_config(config_cls, name_field, name, params):
    # 外层名称是设备关联标识，禁止内部再声明另一个名称。
    if name_field in params:
        raise ValueError(f"{name_field} 请通过 build_* 的 name 参数指定")
    common = {f.name: params.pop(f.name) for f in fields(config_cls) if f.name in params}
    return config_cls(**{name_field: name}, **common)


def build_arm(name: str, spec: Mapping[str, Any]) -> "ArmBase":
    """构建机械臂；支持 rm65（RM65-B，使用 ArmRM65 驱动）。"""
    kind, params = _parse(spec)
    if kind == "rm65":
        from .hardware.arm import ArmConfig, ArmRM65
        config = _device_config(ArmConfig, "arm_name", name, params)
        return ArmRM65(config, **params)
    else:
        logger.error(f"arms.{name}: 不支持的类型 {kind!r}，可选：rm65")
        return None


def build_end_effector(name: str, spec: Mapping[str, Any]) -> "EndEffectorBase":
    """构建末端执行器；支持 lg603。"""
    kind, params = _parse(spec)
    if kind == "lg603":
        from .hardware.end_effector import EndEffectorConfig, HandLG603
        config = _device_config(EndEffectorConfig, "end_effector_name", name, params)
        return HandLG603(config, **params)
    else:
        logger.error(f"end_effectors.{name}: 不支持的类型 {kind!r}, 可选: lg603")
        return None


def build_camera(name: str, spec: Mapping[str, Any]) -> "CameraBase":
    """构建相机；暂支持 orbbec"""
    kind, params = _parse(spec)
    if kind == "orbbec":
        from .hardware.camera import CameraConfig, CameraOrbbec
        config = _device_config(CameraConfig, "camera_name", name, params)
        return CameraOrbbec(config, **params)
    else:
        logger.error(f"cameras.{name}: 不支持的类型 {kind!r}, 可选: orbbec")
        return None


def build_controller(name: str, spec: Mapping[str, Any]) -> "ControllerBase":
    """构建输入设备（非 RobotController）；支持 pico。"""
    kind, params = _parse(spec)
    if kind == "pico":
        from .hardware.controller import ControllerConfig, ControllerPico
        config = _device_config(ControllerConfig, "controller_name", name, params)
        return ControllerPico(config, **params)
    else:
        logger.error(f"controllers.{name}: 不支持的类型 {kind!r}，可选：pico")
        return None


def build_ik(spec: Mapping[str, Any]) -> "PlacoIK":
    """构建运动学求解器；支持 rm65, placo, 构造时加载模型。"""
    kind, params = _parse(spec)
    if kind == "rm65":
        from .kinematics import RM65IK, RM65IKConfig
        return RM65IK(RM65IKConfig(**params))
    elif kind == "placo":
        from .kinematics import PlacoIK, PlacoIKConfig
        return PlacoIK(PlacoIKConfig(**params))
    else:
        logger.error(f"kinematics: 不支持的类型 {kind!r}, 可选: rm65, placo")
