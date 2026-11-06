"""独立 Web 进程：所有页面数据和操作均通过 ROS2 service 请求遥操作节点。"""
import argparse
import base64
import json
from pathlib import Path
import runpy
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParametersAtomically

from web_dashboard import DashboardServer


class DashboardClient(Node):
    def __init__(self, prefix):
        super().__init__('pico_rm65_web')
        self.client = self.create_client(SetParametersAtomically, f"/{prefix.strip('/')}/dashboard")
        self.lock = threading.Lock()

    def request(self, action, payload=None):
        if not self.client.wait_for_service(timeout_sec=1.):
            return {'ok': False, 'message': '遥操作 ROS2 service 未就绪'}
        request = SetParametersAtomically.Request(parameters=[Parameter(
            name=action, value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                             string_value=json.dumps(payload or {}, ensure_ascii=False)))])
        # Executor spins on a separate thread; HTTP handlers only wait for their own future.
        with self.lock:
            future = self.client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(15.):
            future.cancel()
            return {'ok': False, 'message': 'ROS2 service 响应超时；请刷新确认当前状态'}
        if future.cancelled():
            return {'ok': False, 'message': 'ROS2 请求已取消'}
        try:
            return json.loads(future.result().result.reason)
        except Exception as exc:
            return {'ok': False, 'message': f'ROS2 service 失败：{exc}'}

    def status(self):
        result = self.request('status')
        return {'overall': 'offline', 'devices': {}, 'recording': False,
                'recording_duration_sec': 0., 'review_pending': False, **result}

    def image(self, name):
        result = self.request('image', {'name': name})
        return base64.b64decode(result['image']) if result.get('ok') else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--prefix', default='/vr_teleop')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[2] / 'configs/record_pico.py')
    args, ros_args = parser.parse_known_args()
    config = runpy.run_path(str(args.config.expanduser().resolve()))['CONFIG']
    rclpy.init(args=ros_args)
    client = DashboardClient(args.prefix)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(client)
    worker = threading.Thread(target=executor.spin, name='web-ros2', daemon=True)
    worker.start()
    dashboard = None
    try:
        dashboard = DashboardServer(args.host, args.port, list(config['cameras']),
            client.status, client.image,
            lambda metadata=None: client.request('start', metadata),
            lambda: client.request('stop'),
            lambda status, reason, note: client.request('review', {
                'status': status, 'reason': reason, 'note': note}),
            5., lambda: client.request('history'),
            lambda directory: client.request('delete', {'directory': directory}))
        dashboard.start()
        print(f'Web 界面：http://{args.host}:{dashboard.address[1]}', flush=True)
        while rclpy.ok():
            threading.Event().wait(.5)
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard is not None:
            dashboard.stop()
        executor.shutdown()
        worker.join()
        client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
