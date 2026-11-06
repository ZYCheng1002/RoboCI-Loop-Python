"""无需第三方框架的采集控制与质检页面。"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable
from urllib.parse import unquote, urlsplit


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
        submit_review: Callable[[str, str, str], dict],
        preview_rate_hz: float,
        history_provider: Callable[[], dict] | None = None,
        delete_record: Callable[[object], dict] | None = None,
    ) -> None:
        page = self._page(camera_names, preview_rate_hz).encode("utf-8")

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

    @staticmethod
    def _page(camera_names: list[str], preview_rate_hz: float) -> str:
        # 设备标识只用于请求路径，界面统一显示中文。
        camera_labels = {"base": "上方相机", "left_wrist": "左腕相机", "right_wrist": "右腕相机"}
        cameras = "".join(
            f'<figure class="{"main-camera" if name == "base" else "arm-camera"}">'
            f'<img id="cam-{name}" alt="{camera_labels.get(name, "相机")}">'
            f'<figcaption>{camera_labels.get(name, "相机")}</figcaption></figure>'
            for name in camera_names
        )
        interval_ms = max(100, round(1000.0 / preview_rate_hz))
        image_script = "\n".join(
            (
                f'(() => {{ const image = document.getElementById("cam-{name}");\n'
                f' const load = () => image.src = "camera/{name}?t=" + Date.now();\n'
                f' image.onload = image.onerror = () => setTimeout(load, {interval_ms}); '
                'load(); })();'
            )
            for name in camera_names
        )
        return f"""<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>VR Teleop · RM65 采集工作站</title>
<style>
:root{{color-scheme:light;font-family:"IBM Plex Sans","Noto Sans SC",system-ui,sans-serif;font-size:clamp(12px,1vw,14px);--navy:#173f6b;--blue:#2468a2;--blue-2:#eaf3fb;--blue-3:#d5e6f4;--ink:#172536;--muted:#66788b;--line:#cfdae5;--surface:#ffffff;--canvas:#f3f7fa;--green:#16835f;--red:#c8423b}}
*{{box-sizing:border-box}}[hidden]{{display:none!important}}
body{{margin:0;background:var(--canvas);color:var(--ink);background-image:linear-gradient(#173f6b08 1px,transparent 1px),linear-gradient(90deg,#173f6b08 1px,transparent 1px);background-size:32px 32px}}
body::before{{content:"";position:fixed;inset:0 0 auto;height:3px;background:linear-gradient(90deg,var(--navy),#3987c2 65%,#9ec7e7);z-index:10}}
main{{height:100vh;height:100dvh;min-height:620px;padding:12px;display:flex;flex-direction:column;gap:8px;max-width:1800px;margin:auto}}
header,.panel{{background:var(--surface);border:1px solid var(--line);border-radius:4px;padding:10px;min-width:0;box-shadow:0 3px 12px #173f6b0d}}
header{{display:flex;align-items:center;justify-content:space-between;gap:16px;flex-shrink:0;border-left:4px solid var(--navy);padding:9px 12px}}
.brand{{display:flex;align-items:center;gap:14px;min-width:0}}.brand-mark{{width:10px;height:10px;background:var(--blue);box-shadow:0 0 0 5px var(--blue-2);transform:rotate(45deg);flex:none}}
.eyebrow{{font:600 10px/1.2 ui-monospace,SFMono-Regular,monospace;letter-spacing:.18em;color:var(--blue);text-transform:uppercase}}
h1{{font-size:17px;line-height:1.2;margin:2px 0 0;letter-spacing:.02em;color:var(--navy)}}h2{{font:600 11px/1.2 ui-monospace,SFMono-Regular,monospace;letter-spacing:.12em;text-transform:uppercase;margin:0 0 8px;color:var(--navy)}}
.system-state{{font-size:11px;color:var(--muted);margin-top:3px}}#overall{{font-family:ui-monospace,SFMono-Regular,monospace;text-transform:uppercase}}
.grid{{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(280px,.7fr);gap:8px;flex:1.5 1 0;min-height:190px}}
.grid>.panel{{display:flex;flex-direction:column;min-height:0}}
.panel{{position:relative}}.panel::after{{content:"";position:absolute;left:9px;top:-1px;width:38px;height:2px;background:var(--blue)}}
.cameras{{display:grid;grid-template-columns:minmax(0,2.2fr) minmax(150px,1fr);grid-template-rows:repeat(2,minmax(0,1fr));gap:5px;flex:1;min-height:0}}
figure{{margin:0;min-height:0;position:relative;background:#0b1118;border:1px solid #b8c8d6;border-radius:2px;overflow:hidden}}
figure::after{{content:"";position:absolute;inset:5px;pointer-events:none;border-top:1px solid #ffffff30;border-left:1px solid #ffffff30}}
img{{display:block;width:100%;height:100%;object-fit:contain}}
.cameras .main-camera{{grid-row:1 / -1;border-color:#7fa8c8}}
figcaption{{position:absolute;left:7px;top:7px;bottom:auto;background:#ffffffeb;border-left:3px solid var(--blue);color:var(--navy);padding:4px 7px;font:600 10px/1.2 ui-monospace,SFMono-Regular,monospace;letter-spacing:.06em;text-transform:uppercase;box-shadow:0 1px 5px #0002}}
table{{width:100%;border-collapse:collapse;font-size:12px}}td,th{{padding:6px 8px;border-bottom:1px solid #dce5ed;text-align:left}}th{{font:600 10px/1.2 ui-monospace,SFMono-Regular,monospace;letter-spacing:.07em;color:#587087;text-transform:uppercase}}tbody tr:hover{{background:#f2f7fb}}
.online{{color:var(--green)}}.offline{{color:var(--red)}}.recording{{color:var(--red)}}
button,select,input{{font:inherit;border-radius:3px;border:1px solid #b9c9d7;padding:6px 10px;max-width:100%;background:#fff;color:var(--ink)}}
button{{cursor:pointer;background:#edf3f8;color:var(--navy);font:600 11px/1.2 ui-monospace,SFMono-Regular,monospace;letter-spacing:.04em;text-transform:uppercase;transition:background .15s,border-color .15s,transform .15s}}button:hover:not(:disabled){{background:#dfeefa;border-color:#79a6c8}}button:active:not(:disabled){{transform:translateY(1px)}}button.stop{{background:#fff1f0;border-color:#e3aaa6;color:#a72f2a}}button:disabled{{opacity:.4;cursor:not-allowed}}
#start{{background:var(--blue);border-color:var(--blue);color:#fff}}#start:hover:not(:disabled){{background:var(--navy);border-color:var(--navy)}}#timer{{font:600 12px/1 ui-monospace,SFMono-Regular,monospace;padding:7px 10px;border:1px solid #c6d5e1;background:#f7fafc;min-width:110px;text-align:center}}
.controls{{display:flex;align-items:center;gap:6px;flex-wrap:wrap}}
#message{{color:#9a5c17;font-family:ui-monospace,SFMono-Regular,monospace;min-height:0}}#message:empty,#point-message:empty,#history-note:empty{{display:none}}
#point-panel,#review{{flex-shrink:0}}#point-panel input{{width:65px}}
#point-panel .identity-fields{{display:grid;grid-template-columns:2fr 1fr 1fr;gap:10px;margin-bottom:9px}}
.identity-fields label{{display:flex;align-items:center;gap:8px;color:#5c7084;min-width:0;white-space:nowrap;font-size:11px;text-transform:uppercase;letter-spacing:.03em}}
#point-panel .identity-fields input{{width:100%;min-width:0;background:#fbfdff;color:var(--ink);padding:7px 9px}}
input:focus,select:focus{{outline:2px solid #9bc4e3;outline-offset:1px;border-color:var(--blue)}}
.point-controls{{padding-top:8px;border-top:1px solid #d8e2ea}}.point-controls label{{display:flex;align-items:center;gap:6px;font-family:ui-monospace,SFMono-Regular,monospace}}
@media(max-width:520px){{#point-panel .identity-fields{{grid-template-columns:1fr}}}}
#point-hint{{font-size:11px;color:#718397;margin:6px 0 0;line-height:1.45}}
#history{{display:flex;flex-direction:column;gap:6px;flex:1 1 0;min-height:145px}}#history h2{{margin:0}}
.table-scroll{{min-height:0;overflow:auto;border:1px solid #d2dee8;border-radius:3px;flex:1;background:#fff}}
.table-scroll table{{border-collapse:separate;border-spacing:0}}.table-scroll th{{position:sticky;top:0;z-index:1;background:#edf4f9;white-space:nowrap}}
.table-scroll:focus-visible{{outline:2px solid #9bc4e3;outline-offset:1px}}#topics td:first-child{{overflow-wrap:anywhere;font-family:ui-monospace,SFMono-Regular,monospace;color:#435d74}}
#history td{{vertical-align:top;white-space:pre-wrap;overflow-wrap:anywhere;min-width:80px}}#history td:first-child{{min-width:230px;font-family:ui-monospace,SFMono-Regular,monospace;color:#435d74}}#history-note{{color:#9a5c17;margin:0}}
@media(max-width:760px){{main{{height:auto;min-height:100dvh;padding:6px;gap:6px}}header{{flex-wrap:wrap}}.brand{{width:100%}}.grid{{grid-template-columns:1fr;flex:none}}.cameras{{height:clamp(300px,80vw,480px);flex:none;grid-template-columns:repeat(2,minmax(0,1fr));grid-template-rows:minmax(0,2fr) minmax(0,1fr)}}.cameras .main-camera{{grid-row:auto;grid-column:1 / -1}}.grid .table-scroll{{max-height:220px;flex:auto}}#history{{flex:none}}#history .table-scroll{{max-height:240px;flex:auto}}}}
</style>
<main><header><div class="brand"><span class="brand-mark"></span><div><div class="eyebrow">VR TELEOP / DATA STATION</div><h1>RM65 双臂采集工作站</h1><div class="system-state">SYSTEM <b id="overall">等待数据</b></div></div></div>
<div class="controls"><span id="timer">未录制</span><button id="start" onclick="record('start')">开始录制</button>
<button id="stop" class="stop" onclick="record('stop')">结束录制</button></div></header>
<div id="message"></div>
<section id="point-panel" class="panel"><h2>采集参数 / Capture Setup</h2><div class="identity-fields">
<label>Task <input id="task" type="text" value="" autocomplete="off" required maxlength="64" placeholder="例如：抓苹果"></label>
<label>操作员 <input id="operator" class="person-input" type="text" value="" autocomplete="off" required placeholder="必填"></label>
<label>记录员 <input id="recorder" class="person-input" type="text" value="" autocomplete="off" required placeholder="必填"></label>
</div><div class="controls point-controls"><label>x <input id="point-x" type="number" min="0" max="10" step="1" value="0" onchange="savePoint()"></label>
<label>y <input id="point-y" type="number" min="0" max="10" step="1" value="0" onchange="savePoint()"></label>
<button id="next-point" onclick="nextPoint()">下一点位</button>
<label>θ（度） <input id="point-theta" type="number" min="0" max="359.999999" step="any" value="0" onchange="savePoint()"></label>
<button onclick="nextAngle()">角度 +45°</button><button onclick="resetPoint()">归零</button>
</div><p id="point-hint">点位：先递增 y，超过 10 后 x 加 1、y 归零，直到 (10,10)。角度按 360° 循环。
数值可手动输入；开始录制时保存当前 Task、人员、点位和角度，后续修改用于下一条录制。</p>
<div id="point-message" role="status"></div></section>
<div class="grid"><section class="panel"><h2>视觉监控 / Vision</h2>
<div class="cameras">{cameras}</div></section><section class="panel"><h2>设备遥测 / Telemetry</h2>
<div class="table-scroll" tabindex="0" role="region" aria-label="设备频率，可滚动"><table><thead><tr><th>设备</th><th>状态</th><th>频率（Hz）</th></tr></thead><tbody id="topics"></tbody></table></div>
</section></div><section id="review" class="panel" hidden><h2>数据质检 / Review</h2><div class="controls">
<select id="review-status"><option value="accepted">保留</option><option value="rejected">废弃</option>
<option value="needs_review">待检查</option></select><select id="review-reason">
<option value="none">无问题</option><option value="operation_failed">操作失败</option>
<option value="arm_issue">机械臂异常</option><option value="hand_issue">灵巧手异常</option>
<option value="grasp_hesitation">抓取犹豫</option>
<option value="other">其他</option></select><input id="review-note" placeholder="备注（可选）">
<button onclick="submitReview()">提交质检</button></div></section>
<section id="history" class="panel"><h2>采集档案 / Archive</h2>
<div class="controls">
<select id="history-filter" aria-label="筛选质检状态" onchange="renderHistory()">
<option value="all">全部状态</option><option value="accepted">保留</option>
<option value="rejected">废弃</option><option value="needs_review">待检查</option>
<option value="unreviewed">未标注</option><option value="error">读取异常</option></select>
<button onclick="refreshHistory()">刷新历史</button><span id="history-count"></span></div>
<div id="history-note" role="status"></div>
<div class="table-scroll" tabindex="0" role="region" aria-label="历史记录，可上下及左右滚动"><table>
<thead><tr><th>记录</th><th>质检状态</th>
<th>Task</th><th>操作员</th><th>记录员</th><th>点位 (x, y)</th><th>θ（度）</th><th>问题原因</th><th>备注</th><th>操作</th></tr></thead><tbody id="history-rows"></tbody>
</table></div></section></main><script>
function readPoint(){{
 const values=['x','y','theta'].map(name=>document.getElementById('point-'+name));
 for(const input of values){{
  if(input.value===''||!Number.isFinite(input.valueAsNumber)||!input.checkValidity()){{
   input.reportValidity();document.getElementById('point-message').textContent=
    'x、y 请输入 0–10 的整数，θ 请输入 0（含）至 360（不含）的角度。';return null;
  }}
 }}
 return Object.fromEntries(values.map((input,index)=>[['x','y','theta'][index],input.valueAsNumber]));
}}
function savePoint(){{
 const point=readPoint();if(!point)return;
 document.getElementById('next-point').disabled=point.x===10&&point.y===10;
 document.getElementById('point-message').textContent=point.x===10&&point.y===10?'已到最后一个点位':'';
 try{{localStorage.setItem('xr-teleop-point',JSON.stringify(point));}}
 catch(error){{document.getElementById('point-message').textContent='浏览器无法保存数值，刷新后将重置。';}}
}}
function nextPoint(){{
 const point=readPoint();if(!point||point.x===10&&point.y===10)return;
 if(point.y<10)point.y+=1;else{{point.x+=1;point.y=0;}}
 document.getElementById('point-x').value=point.x;
 document.getElementById('point-y').value=point.y;savePoint();
}}
function nextAngle(){{
 const point=readPoint();if(!point)return;
 document.getElementById('point-theta').value=Number(((point.theta+45)%360).toFixed(6));savePoint();
}}
function resetPoint(){{
 for(const name of ['x','y','theta'])document.getElementById('point-'+name).value=0;
 savePoint();
}}
try{{
 const point=JSON.parse(localStorage.getItem('xr-teleop-point'));
 if(point&&Number.isInteger(point.x)&&point.x>=0&&point.x<=10&&
  Number.isInteger(point.y)&&point.y>=0&&point.y<=10&&
  Number.isFinite(point.theta)&&point.theta>=0&&point.theta<360){{
  for(const name of ['x','y','theta'])document.getElementById('point-'+name).value=point[name];
 }}
}}catch(error){{/* 无可恢复的浏览器数据时使用初始值。 */}}
savePoint();
const statuses={{accepted:'保留',rejected:'废弃',needs_review:'待检查',unreviewed:'未标注',error:'读取异常'}};
const reasons={{none:'无问题',operation_failed:'操作失败',arm_issue:'机械臂异常',
 hand_issue:'灵巧手异常',grasp_hesitation:'抓取犹豫',other:'其他'}};
let historyRecords=[];
let historyLoading=false;
const deletingRecords=new Set();
function renderHistory(){{
 const filter=document.getElementById('history-filter').value;
 const rows=document.getElementById('history-rows');rows.replaceChildren();
 const records=historyRecords.filter(item=>{{
  const state=item.error?'error':(item.review?.status||'unreviewed');
  return filter==='all'||state===filter;
 }});
 document.getElementById('history-count').textContent=`显示 ${{records.length}} / ${{historyRecords.length}} 条`;
 for(const item of records){{
  const review=item.review||{{}};const row=document.createElement('tr');
  const state=item.error?'error':(review.status||'unreviewed');
  const values=[item.id,
   statuses[state]||state,item.task||'未记录',item.operator||'未记录',item.recorder||'未记录',
   item.point?`(${{item.point.x}}, ${{item.point.y}})`:'未记录',
   item.point?`${{item.point.theta}}°`:'未记录',reasons[review.reason]||review.reason||'—',
   item.error||review.note||'—'];
  for(const value of values){{const cell=document.createElement('td');cell.textContent=value;row.append(cell)}}
  const action=document.createElement('td');const button=document.createElement('button');
  button.type='button';button.className='stop';
  button.disabled=deletingRecords.has(item.directory);
  button.textContent=button.disabled?'删除中…':'删除';
  button.setAttribute('aria-label','删除记录 '+item.id);
  button.onclick=()=>deleteHistoryRecord(item);action.append(button);row.append(action);
  rows.append(row);
 }}
 if(!records.length){{const row=document.createElement('tr');const cell=document.createElement('td');
 cell.colSpan=10;cell.textContent=historyRecords.length?'没有符合筛选条件的记录':'暂无历史采集记录';
 row.append(cell);rows.append(row)}}
}}
async function deleteHistoryRecord(item){{
 if(deletingRecords.has(item.directory))return;
 if(!confirm(`确定永久删除记录 ${{item.id}}？\nTask：${{item.task||'未记录'}}\n目录：${{item.directory}}\n将删除全部数据文件和标注，无法恢复。`))return;
 deletingRecords.add(item.directory);renderHistory();
 try{{
  const response=await fetch('/api/history/delete',{{method:'POST',
   headers:{{'Content-Type':'application/json'}},body:JSON.stringify({{directory:item.directory}})}});
  const data=await response.json();
  if(!response.ok||!data.ok)throw new Error(data.message||'删除失败');
  historyRecords=historyRecords.filter(record=>record.directory!==item.directory);
  document.getElementById('message').textContent=data.message;
  await refreshHistory();await refresh();
 }}catch(error){{document.getElementById('message').textContent=error.message||'删除失败，请刷新后重试';}}
 finally{{deletingRecords.delete(item.directory);renderHistory();}}
}}
async function refreshHistory(){{
 if(historyLoading)return;historyLoading=true;
 try{{const response=await fetch('/api/history');if(!response.ok)throw new Error('历史记录读取失败');
 const data=await response.json();historyRecords=data.records;renderHistory();
 document.getElementById('history-note').textContent='';
 }}catch(error){{document.getElementById('history-note').textContent='历史记录读取失败，请点击刷新重试。';}}
 finally{{historyLoading=false;}}
}}
async function post(url,body){{
 const response=await fetch(url,{{method:'POST',headers:{{'Content-Type':'application/json'}},
 body:body?JSON.stringify(body):null}}); const data=await response.json();
 document.getElementById('message').textContent=data.message||'';
 await refresh(); await refreshHistory(); return data;
}}
async function record(action){{
 if(action==='start'){{
 const point=readPoint();if(!point)return;
 const metadata={{point}};
 for(const name of ['task','operator','recorder']){{
  const input=document.getElementById(name);input.value=input.value.trim();
  if(!input.reportValidity())return;
  metadata[name]=input.value;
 }}
 await post('/api/record/start',metadata);
 }}
 else await post('/api/record/'+action);
}}
async function submitReview(){{await post('/api/review',{{status:document.getElementById('review-status').value,
 reason:document.getElementById('review-reason').value,note:document.getElementById('review-note').value}})}}
async function refresh(){{const data=await (await fetch('/api/status')).json();
 const overall=document.getElementById('overall');overall.textContent=data.overall==='online'?'全部在线':'部分离线';
 overall.className=data.overall==='online'?'online':'offline';
 document.getElementById('start').disabled=data.recording;document.getElementById('stop').disabled=!data.recording;
 document.getElementById('timer').textContent=data.recording?'● 录制 '+data.recording_duration_sec.toFixed(1)+' s':'未录制';
 document.getElementById('timer').className=data.recording?'recording':'';
 document.getElementById('review').hidden=!data.review_pending;
 document.getElementById('topics').innerHTML=Object.entries(data.devices).map(([name,item])=>
 `<tr><td>${{name}}</td><td class="${{item.online?'online':'offline'}}">${{item.online?'在线':'离线'}}</td>`+
 `<td>${{item.fps.toFixed(1)}}</td></tr>`).join('');}}
function startImageRefresh(){{{image_script}}}
setInterval(refresh,1000);refresh();startImageRefresh();
setInterval(refreshHistory,10000);refreshHistory();
</script></html>"""
