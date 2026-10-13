"""Offline numerical logs and snapshots. No ROS subscriptions or execution of files."""
from dataclasses import dataclass
from pathlib import Path
from datetime import datetime, timezone
import csv
import json
import re
import shutil
import numpy as np


@dataclass
class Dataset:
    path: Path
    time: np.ndarray
    fields: dict
    time_unit: str = 'sample'
    has_time: bool = False
    xyz: tuple | None = None


def flatten(data, prefix=''):
    result = {}
    for name, value in data.items():
        key = f'{prefix}.{name}' if prefix else name
        if isinstance(value, dict):
            result.update(flatten(value, key))
        elif isinstance(value, (int, float, bool)):
            result[key] = float(value)
        elif isinstance(value, list) and len(value) <= 16 and all(isinstance(v, (int, float)) for v in value):
            result.update({f'{key}[{i}]': float(v) for i, v in enumerate(value)})
    return result


def number(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return np.nan


def normal(name):
    return re.sub(r'\[.*?\]|\(.*?\)', '', name.lower()).strip().replace(' ', '_')


def load_numeric(path, time_unit='auto'):
    path = Path(path)
    if path.suffix.lower() in ('.json', '.jsonl'):
        if path.suffix.lower() == '.jsonl':
            records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        else:
            source = json.loads(path.read_text())
            records = source if isinstance(source, list) else [source]
        rows = [flatten(row) for row in records if isinstance(row, dict)]
        headers = list(dict.fromkeys(key for row in rows for key in row))
        matrix = np.array([[row.get(key, np.nan) for key in headers] for row in rows], dtype=float)
    else:
        lines = [line.strip() for line in path.read_text().splitlines() if line.strip()]
        if not lines:
            raise ValueError('빈 파일입니다.')
        comment_header = next((line.lstrip('#').strip() for line in reversed(lines) if line.startswith('#') and 'timestamp' in line.lower()), None)
        lines = [line for line in lines if not line.startswith('#')]
        if not lines:
            raise ValueError('수치 기록이 없습니다.')
        def split(line):
            if ',' in line:
                return next(csv.reader([line]))
            if ';' in line:
                return next(csv.reader([line], delimiter=';'))
            return line.split()
        def clean(line):
            values = split(line)
            while values and not values[-1].strip():
                values.pop()
            return values
        first = clean(lines[0])
        def is_number(value):
            try:
                float(value)
                return True
            except ValueError:
                return False
        header = any(not is_number(value) for value in first)
        if header:
            headers = first
            lines = lines[1:]
        elif path.suffix.lower() == '.tum':
            headers = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'][:len(first)]
        elif comment_header and len(clean(comment_header)) == len(first):
            headers = clean(comment_header)
        else:
            # Headerless VIO files: first four columns are time,x,y,z. Other fields remain unnamed.
            headers = ['timestamp', 'x', 'y', 'z'][:len(first)] + [f'column_{i+1}' for i in range(4, len(first))]
        rows = [clean(line) for line in lines]
        if any(len(row) != len(headers) for row in rows):
            raise ValueError('행마다 열 수가 다릅니다.')
        matrix = np.array([[number(value) for value in row] for row in rows], dtype=float)
    if not matrix.size:
        raise ValueError('표시할 수치가 없습니다.')
    # Duplicate field names must not silently replace columns.
    names = []
    for key in headers:
        candidate = str(key).strip()
        if candidate in names:
            candidate = f'{candidate}_{len(names)}'
        names.append(candidate)
    fields = {key: matrix[:, i] for i, key in enumerate(names) if np.isfinite(matrix[:, i]).any()}
    if not fields:
        raise ValueError('표시할 수치가 없습니다.')
    aliases = {normal(key): key for key in fields}
    time_key = next((aliases[key] for key in ('timestamp', 'time', 't', 'stamp', 'timestamp_ns', 'timestamp_s', 'time_s', 't_s') if key in aliases), None)
    time = fields.pop(time_key).copy() if time_key else np.arange(len(matrix), dtype=float)
    used_unit = 'sample'
    if time_key:
        used_unit = time_unit
        if used_unit == 'auto':
            valid = np.sort(time[np.isfinite(time)])
            gaps = np.diff(valid)
            gaps = gaps[gaps > 0]
            typical = np.median(gaps) if gaps.size else 0
            magnitude = np.median(np.abs(valid)) if valid.size else 0
            used_unit = 'ns' if normal(time_key).endswith('_ns') or typical > 1e6 or magnitude > 1e16 else 'us' if magnitude > 1e14 else 'ms' if magnitude > 1e11 else 's'
        time /= {'s': 1, 'ms': 1e3, 'us': 1e6, 'ns': 1e9}[used_unit]
    valid = np.isfinite(time)
    order = np.argsort(time[valid], kind='stable')
    time = time[valid][order]
    fields = {key: values[valid][order] for key, values in fields.items()}
    xyz = None
    aliases = {normal(key): key for key in fields}
    for prefix in ('', 'p_', 'pos_', 'position_', 'position.', 'pose.position.', 'pose.pose.position.'):
        keys = tuple(aliases.get(prefix + axis) for axis in 'xyz')
        if all(keys):
            xyz = keys
            break
    if time.size == 0:
        raise ValueError('유효한 시간 값이 없습니다.')
    return Dataset(path, time, fields, used_unit, time_key is not None, xyz)


def associate(reference, estimate, max_gap=.3):
    if not reference.xyz or not estimate.xyz or not reference.has_time or not estimate.has_time:
        raise ValueError('정렬에는 시간과 x,y,z가 있는 두 파일이 필요합니다.')
    a = np.column_stack([reference.fields[key] for key in reference.xyz])
    b = np.column_stack([estimate.fields[key] for key in estimate.xyz])
    good = np.isfinite(a).all(axis=1)
    rt, a = reference.time[good], a[good]
    unique = np.r_[True, np.diff(rt) > 0]
    rt, a = rt[unique], a[unique]
    if len(rt) < 2:
        raise ValueError('기준 궤적에 유효한 시각이 부족합니다.')
    index = np.searchsorted(rt, estimate.time)
    left = np.clip(index-1, 0, len(rt)-1)
    right = np.clip(index, 0, len(rt)-1)
    valid = (estimate.time >= rt[0]) & (estimate.time <= rt[-1]) & np.isfinite(b).all(axis=1)
    exact = (np.abs(estimate.time-rt[left]) < 1e-8) | (np.abs(estimate.time-rt[right]) < 1e-8)
    valid &= exact | ((rt[right]-rt[left]) <= max_gap)
    interp = np.column_stack([np.interp(estimate.time[valid], rt, a[:, i]) for i in range(3)])
    return valid, interp, b[valid]


def rigid_alignment(reference, estimate, max_gap=.3):
    mask, target, source = associate(reference, estimate, max_gap)
    if len(source) < 3:
        raise ValueError('겹치는 위치 기록이 3개 미만입니다.')
    centered = source-source.mean(axis=0)
    if np.linalg.matrix_rank(centered, tol=1e-8) < 2:
        raise ValueError('직선 또는 정지 궤적만으로는 전체 3D 회전을 결정할 수 없습니다.')
    u, _, vt = np.linalg.svd(centered.T@(target-target.mean(axis=0)))
    rotation = vt.T@u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T@u.T
    translation = target.mean(axis=0)-rotation@source.mean(axis=0)
    points = np.column_stack([estimate.fields[key] for key in estimate.xyz])
    aligned = points@rotation.T+translation
    rmse = np.sqrt(np.mean(np.sum((aligned[mask]-target)**2, axis=1)))
    return aligned, float(rmse), int(mask.sum())


def save_session(root, name, console, history, settings, attachments=()):
    root = Path(root)
    safe = re.sub(r'[^\w가-힣.-]+', '_', name).strip('._')[:80] or '실행'
    created = datetime.now(timezone.utc)
    root.mkdir(parents=True, exist_ok=True)
    directory = root/(created.astimezone().strftime('%Y%m%d_%H%M%S_%f')+'_'+safe)
    directory.mkdir()
    manifest = {'schema_version': 1, 'name': name, 'created_at': created.isoformat(), 'runs': history,
                'console': 'console.log', 'files': [], 'kind': 'offline_log_snapshot',
                'collection': 'GUI console and explicitly selected result files; no ROS topic recording'}
    try:
        shutil.copy2(console, directory/'console.log')
        (directory/'settings.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2))
        (directory/'results').mkdir()
        for i, source in enumerate(attachments):
            source = Path(source)
            if not source.is_file():
                raise ValueError(f'결과 파일을 찾을 수 없습니다: {source}')
            target = directory/'results'/f'{i+1:02d}_{source.name}'
            shutil.copy2(source, target)
            manifest['files'].append({'path': str(target.relative_to(directory)), 'source': str(source), 'bytes': target.stat().st_size})
        # Preserve parameter overrides after their original temporary directories disappear.
        for i, run in enumerate(history):
            source = Path(run.get('options_directory', ''))
            if source.is_dir() and run.get('options_directory'):
                target = directory/'config'/f'run_{i+1:02d}'
                for file in source.rglob('*'):
                    if file.is_file() and file.suffix.lower() in ('.yaml', '.json', '.rviz', '.sdf', '.urdf'):
                        dest = target/file.relative_to(source)
                        dest.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(file, dest)
        (directory/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    except Exception:
        shutil.rmtree(directory)
        raise
    return directory


def sessions(root):
    result = []
    for path in Path(root).glob('*/manifest.json'):
        try:
            data = json.loads(path.read_text())
            data['directory'] = path.parent
            result.append(data)
        except (ValueError, OSError):
            continue
    return sorted(result, key=lambda row: row.get('created_at', ''), reverse=True)


def summary_plots(directory):
    # Import pyplot only after GUI exists; never touches live ROS state.
    configure_fonts()
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    datasets, errors, maps = [], [], []
    for item in manifest['files']:
        path = directory/item['path']
        try:
            if path.suffix.lower() in ('.npz', '.png', '.jpg', '.jpeg'):
                maps.append((path, load_snapshot(path)))
                continue
            datasets.append(load_numeric(path))
        except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
            errors.append({'file': path.name, 'reason': str(error)})
    figures = []
    for index,(source,snapshot) in enumerate(maps[:3]):
        figure=Figure(figsize=(12,4),tight_layout=True);FigureCanvasAgg(figure)
        layers=list(snapshot['layers'].items())
        for i,(key,array) in enumerate(layers):
            axes=figure.add_subplot(1,len(layers),i+1)
            image=axes.imshow(array,cmap='tab10' if key=='classes' else 'viridis')
            axes.set_title(key)
            if array.ndim==2:figure.colorbar(image,ax=axes,shrink=.7)
        path=directory/f'map_{index+1:02d}.png';figure.savefig(path,dpi=140);figures.append(path.name)
    trajectories = [data for data in datasets if data.xyz]
    if trajectories:
        figure = Figure(figsize=(8,6), tight_layout=True);FigureCanvasAgg(figure)
        axes = figure.add_subplot(111, projection='3d')
        for data in trajectories:
            points = [data.fields[key] for key in data.xyz]
            axes.plot(*points, label=data.path.stem)
        axes.set(xlabel='X', ylabel='Y', zlabel='Z', title='Recorded trajectories (original coordinates)')
        axes.legend();path=directory/'trajectory.png';figure.savefig(path,dpi=140);figures.append(path.name)
    scalar = [(data,key) for data in datasets for key in data.fields if not data.xyz or key not in data.xyz][:8]
    if scalar:
        figure=Figure(figsize=(11, max(3, 2.3*((len(scalar)+1)//2))), tight_layout=True);FigureCanvasAgg(figure)
        for i,(data,key) in enumerate(scalar):
            axes=figure.add_subplot((len(scalar)+1)//2,2,i+1)
            axes.plot(data.time-data.time[0],data.fields[key],marker='.' if len(data.time)==1 else None);axes.set(title=f'{data.path.stem}: {key}',xlabel='Elapsed time [s]' if data.has_time else 'Sample')
        path=directory/'scalars.png';figure.savefig(path,dpi=140);figures.append(path.name)
    (directory/'plot_report.json').write_text(json.dumps({'figures':figures,'unparsed':errors,'coordinates':'unaligned original file coordinates; verify frame, pivot and units before comparison'},ensure_ascii=False,indent=2))
    return figures


def configure_fonts():
    from matplotlib import rcParams, font_manager
    available={font.name for font in font_manager.fontManager.ttflist}
    family=next((name for name in ('Noto Sans CJK JP','Noto Sans CJK KR','Noto Serif CJK JP','NanumGothic') if name in available),None)
    if family:rcParams['font.family']=[family,'DejaVu Sans']
    rcParams['axes.unicode_minus']=False


def load_snapshot(path):
    path=Path(path)
    if path.suffix.lower()=='.npz':
        with np.load(path,allow_pickle=False) as file:
            layers={key:file[key].copy() for key in file.files if file[key].ndim==2}
            origin=file['origin'].tolist() if 'origin' in file else [0.,0.]
            resolution=float(file['resolution']) if 'resolution' in file else None
        if not layers:raise ValueError('2D 지도 배열이 없습니다.')
        return {'layers':layers,'origin':origin,'resolution':resolution}
    from matplotlib.image import imread
    return {'layers':{'image':imread(path)},'origin':[0.,0.],'resolution':None}


def recover_console_metrics(directory):
    """Recover only numerical values actually printed, not unrecorded trajectories."""
    directory=Path(directory);manifest_path=directory/'manifest.json'
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('console_metrics_processed'):return manifest
    console=directory/manifest.get('console','console.log')
    rows={'semantic':[],'perception':[]};last_time=None
    if console.is_file():
        for line in console.read_text(errors='replace').splitlines():
            times=re.findall(r'\[(\d{8,}\.\d+)\]',line)
            if times:last_time=float(times[-1])
            elif re.match(r'\d{4}-\d\d-\d\dT',line):
                try:last_time=datetime.fromisoformat(line.split()[0]).timestamp()
                except ValueError:pass
            if last_time is None:continue
            match=re.search(r'frames=(\d+),\s*inference=([\d.]+)\s*ms',line)
            if match:rows['semantic'].append([last_time,int(match[1]),float(match[2])])
            metrics=dict(re.findall(r"'(frames|processing_ms|height_cells|slope_cells|class_cells)'\s*:\s*([-+\d.eE]+)",line))
            if 'processing_ms' in metrics:rows['perception'].append([last_time]+[number(metrics.get(key)) for key in ('frames','processing_ms','height_cells','slope_cells','class_cells')])
    # Do not mix wall-time console metrics into normal ROS-time data by default.
    if not manifest.get('files'):
        folder=directory/'results';folder.mkdir(exist_ok=True)
        for kind,data in rows.items():
            if not data:continue
            file=folder/f'{kind}_console.csv'
            headers=['time','frames','inference_ms'] if kind=='semantic' else ['time','frames','processing_ms','height_cells','slope_cells','class_cells']
            with file.open('w',newline='') as stream:
                writer=csv.writer(stream);writer.writerow(headers);writer.writerows(data)
            manifest['files'].append({'path':str(file.relative_to(directory)),'derived_from':'console.log','time_basis':'wall_time','bytes':file.stat().st_size})
    original=directory/'manifest.before_console_metrics.json'
    if not original.exists():shutil.copy2(manifest_path,original)
    manifest['console_metrics_processed']=True
    manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return manifest


def discover_outputs(history):
    result=[]
    for run in history:
        root=Path(run.get('output_directory',''))
        if not run.get('output_directory') or not root.is_dir():continue
        result.extend(path for path in root.rglob('*') if path.is_file() and (path.suffix in ('.csv','.npz') or path.name.startswith('estimator_epoch_') and path.suffix=='.log') and '.pending.' not in path.name)
    return sorted(set(result))
