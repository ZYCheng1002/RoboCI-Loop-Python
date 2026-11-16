"""无需第三方框架的采集控制与质检页面。"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable
from urllib.parse import unquote, urlsplit

from web_page import build_page


class DashboardServer:
    """提供 设备频率、相机缩略图、录制控制和质检接口。"""

    def __init__(
        self,
        host: str,
        port: int,
        camera_names: list[str],
        status_provider: Callable[[], dict],
        image_provider: Callable[[str], bytes | None],
        start_recording: Callable[..., dict],
        stop_recording: Callable[[], dict],
        start_inference: Callable[[str, str], dict],
        stop_inference: Callable[[], dict],
        rollback: Callable[[], dict],
        abort_inference: Callable[[], dict],
        review_inference: Callable[[bool], dict],
        go_home: Callable[[], dict],
        prompts: list[str],
        submit_review: Callable[[str, str, str], dict],
        preview_rate_hz: float,
        history_provider: Callable[[], dict] | None = None,
        delete_record: Callable[[object], dict] | None = None,
    ) -> None:
        page = build_page(camera_names, prompts, preview_rate_hz).encode("utf-8")

        class Handler(BaseHTTPRequestHandler):
            def do_GET(handler_self) -> None:  # noqa: N802
                if handler_self.path == "/":
                    handler_self._send(200, "text/html; charset=utf-8", page)
                    return
                if handler_self.path == "/api/status":
                    handler_self._send_json(200, status_provider())
                    return
                if handler_self.path == "/api/history":
                    try:
                        handler_self._send_json(
                            200, history_provider() if history_provider else {"records": []}
                        )
                    except Exception:
                        handler_self._send_json(500, {"message": "历史记录读取失败，请重试"})
                    return
                if handler_self.path.startswith("/camera/"):
                    path = urlsplit(handler_self.path).path
                    name = unquote(path.removeprefix("/camera/"))
                    image = image_provider(name)
                    if image is not None:
                        handler_self._send(200, "image/jpeg", image)
                    else:
                        handler_self._send(404, "text/plain", b"no image")
                    return
                handler_self._send(404, "text/plain", b"not found")

            def do_POST(handler_self) -> None:  # noqa: N802
                try:
                    if handler_self.path == "/api/record/start":
                        if int(handler_self.headers.get("Content-Length", "0")):
                            point = handler_self._read_json()
                            handler_self._send_result(start_recording(point))
                        else:
                            handler_self._send_result(start_recording())
                        return
                    if handler_self.path == "/api/record/stop":
                        handler_self._send_result(stop_recording())
                        return
                    if handler_self.path == "/api/inference/start":
                        data = handler_self._read_json()
                        handler_self._send_result(start_inference(
                            str(data.get("prompt", "")), str(data.get("task", ""))
                        ))
                        return
                    if handler_self.path == "/api/inference/stop":
                        handler_self._send_result(stop_inference())
                        return
                    if handler_self.path == "/api/inference/abort":
                        handler_self._send_result(abort_inference())
                        return
                    if handler_self.path == "/api/inference/review":
                        data = handler_self._read_json()
                        handler_self._send_result(review_inference(data.get("success")))
                        return
                    if handler_self.path == "/api/home":
                        handler_self._send_result(go_home())
                        return
                    if handler_self.path == "/api/rollback":
                        handler_self._send_result(rollback())
                        return
                    if handler_self.path == "/api/history/delete" and delete_record:
                        data = handler_self._read_json()
                        if not isinstance(data, dict):
                            handler_self._send_json(
                                400, {"ok": False, "message": "请求必须是 JSON 对象"}
                            )
                            return
                        handler_self._send_result(delete_record(data.get("directory")))
                        return
                    if handler_self.path == "/api/review":
                        data = handler_self._read_json()
                        result = submit_review(
                            str(data.get("status", "")),
                            str(data.get("reason", "")),
                            str(data.get("note", "")),
                        )
                        handler_self._send_result(result)
                        return
                except (json.JSONDecodeError, UnicodeDecodeError):
                    handler_self._send_json(400, {"ok": False, "message": "JSON 无效"})
                    return
                except Exception as error:
                    handler_self._send_json(
                        500, {"ok": False, "message": f"服务器错误：{error}"}
                    )
                    return
                handler_self._send(404, "text/plain", b"not found")

            def _read_json(handler_self) -> dict:
                length = int(handler_self.headers.get("Content-Length", "0"))
                return json.loads(handler_self.rfile.read(length).decode("utf-8"))

            def _send_result(handler_self, result: dict) -> None:
                handler_self._send_json(200 if result.get("ok") else 409, result)

            def _send_json(handler_self, code: int, value: dict) -> None:
                body = json.dumps(value, ensure_ascii=False).encode("utf-8")
                handler_self._send(code, "application/json; charset=utf-8", body)

            def _send(handler_self, code: int, content_type: str, body: bytes) -> None:
                try:
                    handler_self.send_response(code)
                    handler_self.send_header("Content-Type", content_type)
                    handler_self.send_header("Content-Length", str(len(body)))
                    handler_self.send_header("Cache-Control", "no-store")
                    handler_self.end_headers()
                    handler_self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    # 页面刷新、关闭或替换预览图时，浏览器会主动取消旧请求。
                    handler_self.close_connection = True

            def log_message(self, _format: str, *_args: object) -> None:
                return

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="collector-web", daemon=True
        )

    @property
    def address(self) -> tuple[str, int]:
        return self._server.server_address

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
