"""准备设备反馈/目标事件，不补齐或重采样状态。"""
import copy


class RobotEvents:
    """从启动反馈取得关节顺序，仅为事件补充名称，不缓存状态或重复目标。"""

    def __init__(self, initial):
        self._names = {}
        for kind, attr in (("arms", "arm_name"), ("end_effectors", "end_effector_name")):
            for item in getattr(initial, kind):
                names = list(item.joints.names)
                if len(names) != len(item.joints.position) or len(set(names)) != len(names):
                    raise ValueError(f"{getattr(item, attr)} 关节名称缺失或重复")
                self._names[kind, getattr(item, attr)] = names

    def prepare(self, data, *, kind):
        if kind not in ('state', 'target'):
            raise ValueError('kind 必须为 state 或 target')
        if data is None or not (data.arms or data.end_effectors):
            return None
        data = copy.deepcopy(data)
        for group, attr in (("arms", "arm_name"), ("end_effectors", "end_effector_name")):
            for item in getattr(data, group):
                names = self._names[group, getattr(item, attr)]
                if len(item.joints.position) != len(names):
                    raise ValueError('关节数量变化')
                if item.joints.names and list(item.joints.names) != names:
                    raise ValueError('关节顺序变化')
                item.joints.names = list(names)
                if kind == 'target':
                    item.time_stamp = data.time_stamp
        return data
