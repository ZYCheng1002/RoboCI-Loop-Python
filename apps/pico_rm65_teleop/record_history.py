"""从采集目录读取历史记录及质检标注，无需加载 ROS。"""

import json
import math
import shutil
from datetime import datetime
from pathlib import Path


def validate_point(point: object) -> dict:
    """验证并复制采集点位，角度单位为度。"""
    if not isinstance(point, dict):
        raise ValueError('点位必须包含 x、y、theta')
    for name in ('x', 'y', 'theta'):
        value = point.get(name)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{name} 必须是有限数字')
        if name in ('x', 'y') and (not 0 <= value <= 10 or int(value) != value):
            raise ValueError(f'{name} 必须是 0–10 的整数')
        if name == 'theta' and not 0 <= value < 360:
            raise ValueError('theta 必须在 [0, 360) 度范围内')
    return {name: point[name] for name in ('x', 'y', 'theta')}


def validate_metadata(metadata: object) -> dict:
    """验证录制元信息及用作目录名称的任务。"""
    if not isinstance(metadata, dict):
        raise ValueError('采集信息必须是对象')
    result = {'point': validate_point(metadata.get('point'))}
    for name, label in (('task', 'Task'), ('operator', '操作员'), ('recorder', '记录员')):
        value = metadata.get(name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'请填写{label}')
        result[name] = value.strip()
    task = result['task']
    if len(task) > 64 or any(c in '/\\<>:"|?*' or ord(c) < 32 for c in task):
        raise ValueError('Task 最多 64 个字符，不能包含路径分隔符、控制字符或 <>:"|?*')
    if task in ('.', '..') or task.endswith('.'):
        raise ValueError('Task 不能是 . 或 ..，也不能以句点结尾')
    return result


def read_history(root: Path, active_directory: Path | None = None) -> dict:
    """按时间倒序列出 MCAP 目录，跳过录制中目录。"""
    records = []
    # 同时支持旧版 root/时间戳 与新版 root/Task/时间戳。严格校验时间戳，
    # 避免把以数字开头的 Task 目录误认为一条旧版记录。
    def is_record_directory(path: Path) -> bool:
        try:
            datetime.strptime(path.name, '%Y%m%d_%H%M%S_%f')
            return path.is_dir()
        except ValueError:
            return False

    directories = [
        path for pattern in ('[0-9]*', '*/[0-9]*')
        for path in root.glob(pattern) if is_record_directory(path)
    ]
    for directory in sorted(directories, key=lambda path: path.name, reverse=True):
        if (directory.is_symlink() or directory.parent.is_symlink()
                or not directory.is_dir() or directory == active_directory):
            continue
        record = {
            'id': str(directory.relative_to(root)),
            'directory': str(directory),
            'created_at': None,
            'size_bytes': None,
            'review': None,
            'point': None,
            'task': '',
            'operator': '',
            'recorder': '',
            'error': '',
        }
        try:
            record['created_at'] = datetime.strptime(
                directory.name, '%Y%m%d_%H%M%S_%f'
            ).isoformat()
        except ValueError:
            pass
        try:
            record['size_bytes'] = sum(
                path.stat().st_size for path in directory.glob('*.mcap')
                if not path.is_symlink() and path.is_file()
            )
            review_path = directory / 'review.json'
            if review_path.is_symlink():
                raise ValueError('标注文件不能是符号链接')
            if review_path.exists():
                review = json.loads(review_path.read_text(encoding='utf-8'))
                if not isinstance(review, dict) or any(
                    not isinstance(review.get(key, ''), str)
                    for key in ('status', 'reason', 'note', 'reviewed_at')
                ) or review.get('status') not in {'accepted', 'rejected', 'needs_review'}:
                    raise ValueError('标注格式无效')
                record['review'] = review
        except (OSError, ValueError, UnicodeError) as error:
            record['error'] = f'读取记录失败：{error}'
        try:
            metadata_path = directory / 'metadata.json'
            if metadata_path.is_symlink():
                raise ValueError('元信息文件不能是符号链接')
            if metadata_path.exists():
                metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
                if not isinstance(metadata, dict):
                    raise ValueError('元信息格式无效')
                record['point'] = validate_point(metadata.get('point'))
                for name in ('task', 'operator', 'recorder'):
                    value = metadata.get(name, '')
                    if not isinstance(value, str):
                        raise ValueError('人员信息格式无效')
                    record[name] = value
        except (OSError, ValueError, UnicodeError) as error:
            record['error'] = '; '.join(filter(None, [
                record['error'], f'读取元信息失败：{error}'
            ]))
        records.append(record)
    return {'records': records}


def discard_history_record(
    root: Path, directory: object, active_directory: Path | None = None
) -> Path:
    """将 Task 下已完成的记录移动到该 Task 的 .trash 目录。"""
    if not isinstance(directory, (str, Path)) or not str(directory):
        raise ValueError("没有可废弃的历史记录")
    target = Path(directory)
    resolved_root = root.resolve()
    records = read_history(root, active_directory)["records"]
    listed = {Path(record["directory"]).resolve() for record in records}
    resolved_target = target.resolve()
    if resolved_target not in listed:
        raise ValueError("记录不存在或正在录制，请刷新历史列表")
    try:
        relative = resolved_target.relative_to(resolved_root)
    except ValueError as exc:
        raise ValueError("记录必须位于采集目录内") from exc
    if len(relative.parts) != 2:
        raise ValueError("只有 Task 目录下的记录可以移入废弃目录")
    if active_directory is not None and resolved_target == active_directory.resolve():
        raise ValueError("不能废弃正在录制的记录")

    task_directory = resolved_target.parent
    trash_directory = task_directory / ".trash"
    if trash_directory.is_symlink():
        raise ValueError("废弃目录不能是符号链接")
    trash_directory.mkdir(exist_ok=True)
    destination = trash_directory / resolved_target.name
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"废弃目录中已存在同名记录：{destination.name}")
    resolved_target.rename(destination)
    return destination


def delete_history_record(
    root: Path, directory: object, active_directory: Path | None = None
) -> Path:
    """永久删除已列出的历史记录，拒绝任意路径及正在录制的目录。"""
    if not isinstance(directory, str) or not directory:
        raise ValueError('请选择要删除的历史记录')
    records = read_history(root, active_directory)['records']
    if directory not in {record['directory'] for record in records}:
        raise ValueError('记录不存在或正在录制，请刷新历史列表')
    target = Path(directory)
    resolved_root = root.resolve()
    if not target.resolve().is_relative_to(resolved_root):
        raise ValueError('记录必须位于采集目录内')
    if any(path.is_symlink() for path in (target, *target.parents) if path != root):
        raise ValueError('记录路径不能包含符号链接')
    if active_directory is not None and target.resolve() == active_directory.resolve():
        raise ValueError('不能删除正在录制的记录')
    shutil.rmtree(target)
    # 新目录结构中，最后一条记录删除后不保留空 Task 目录。
    if target.parent != resolved_root:
        try:
            target.parent.rmdir()
        except OSError:
            pass
    return target
