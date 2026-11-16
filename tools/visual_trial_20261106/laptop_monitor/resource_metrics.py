"""Board resource wire contract and bounded history; no SSH, ROS or commands."""
from collections import defaultdict, deque
import math

SOURCES = {"resources_jetson": "jetson", "resources_pi4": "pi4"}
STALE_S = 3.0


def number(value, low=0., high=1e15, nullable=False):
    return ((nullable and value is None) or
            (not isinstance(value, bool) and isinstance(value, (int, float)) and
             math.isfinite(value) and low <= value <= high))


def validate_resource(event, board=None):
    if not isinstance(event, dict):
        raise ValueError("resource object")
    owner = SOURCES.get(event.get("source"))
    if (owner is None or event.get("board") != owner or (board is not None and board != owner)
            or event.get("kind") != "telemetry" or event.get("schema") != 1):
        raise ValueError("resource owner/schema")
    for key in ("t", "sample_monotonic_s"):
        if not number(event.get(key), low=1e-12):
            raise ValueError("resource time")
    for key in ("session", "boot_id", "model", "kernel"):
        if not isinstance(event.get(key), str) or len(event[key]) > 256:
            raise ValueError("resource identity")
    if not event["session"] or not event["boot_id"]:
        raise ValueError("resource session")
    if type(event.get("seq")) is not int or event["seq"] < 1:
        raise ValueError("resource sequence")
    cpu = event.get("cpu")
    if not isinstance(cpu, dict) or cpu.get("state") not in ("warming_up", "live"):
        raise ValueError("resource CPU state")
    if type(cpu.get("online_cores")) is not int or not 1 <= cpu["online_cores"] <= 256:
        raise ValueError("resource CPU count")
    if not number(cpu.get("total_percent"), high=100., nullable=True):
        raise ValueError("resource CPU percent")
    if not number(cpu.get("interval_s"), high=1e9, nullable=True):
        raise ValueError("resource interval")
    if cpu["state"] == "live" and (cpu["total_percent"] is None or
                                    not number(cpu["interval_s"], low=1e-6, high=5.)):
        raise ValueError("resource live window")
    cores = cpu.get("cores")
    if not isinstance(cores, list) or len(cores) != cpu["online_cores"]:
        raise ValueError("resource cores")
    seen = set()
    for c in cores:
        if not isinstance(c, dict) or not isinstance(c.get("id"), str) or not c["id"].startswith("cpu"):
            raise ValueError("resource core ID")
        if c["id"] in seen or len(c["id"]) > 12:
            raise ValueError("resource duplicate core")
        seen.add(c["id"])
        if not number(c.get("percent"), high=100., nullable=True) or not number(c.get("frequency_mhz"), high=20000., nullable=True):
            raise ValueError("resource core values")
    mem = event.get("memory")
    if not isinstance(mem, dict) or any(not number(mem.get(k), nullable=True) for k in
            ("total_mib", "available_mib", "used_mib", "swap_total_mib", "swap_used_mib")):
        raise ValueError("resource memory")
    gpu = event.get("gpu")
    if not isinstance(gpu, dict) or not number(gpu.get("load_percent"), high=100., nullable=True) or not number(gpu.get("frequency_mhz"), high=20000., nullable=True):
        raise ValueError("resource GPU")
    # 2026-11-13: optional, from helpers that read the GPU load ~20x/s (older helpers omit them).
    if "max_percent" in gpu and not number(gpu.get("max_percent"), high=100., nullable=True):
        raise ValueError("resource GPU max")
    if "reads" in gpu and (type(gpu.get("reads")) is not int or not 0 <= gpu["reads"] <= 1000):
        raise ValueError("resource GPU reads")
    temperatures = event.get("temperatures")
    if not isinstance(temperatures, list) or len(temperatures) > 32:
        raise ValueError("resource temperatures")
    for t in temperatures:
        if not isinstance(t, dict) or not isinstance(t.get("name"), str) or len(t["name"]) > 256 or not number(t.get("celsius"), low=-50, high=200):
            raise ValueError("resource temperature")
    rows = event.get("processes")
    if not isinstance(rows, list) or len(rows) > 64:
        raise ValueError("resource process count")
    seen = set()
    for p in rows:
        if not isinstance(p, dict):
            raise ValueError("resource process")
        for key in ("pid", "ppid", "start_ticks", "threads"):
            if type(p.get(key)) is not int or not 0 <= p[key] < 2**63:
                raise ValueError("resource process ID")
        if p["pid"] < 1 or p["pid"] in seen:
            raise ValueError("resource duplicate process")
        seen.add(p["pid"])
        if any(not isinstance(p.get(k), str) or len(p[k]) > 256 for k in ("name", "group")):
            raise ValueError("resource process name")
        if not number(p.get("cpu_percent_one_core"), high=100000., nullable=True) or not number(p.get("rss_mib")):
            raise ValueError("resource process values")
    for key in ("processes_total", "processes_omitted", "proc_read_failures"):
        if type(event.get(key)) is not int or not 0 <= event[key] < 2**31:
            raise ValueError("resource scan count")
    if not number(event.get("scan_ms"), high=1e9):
        raise ValueError("resource scan time")
    return event


def group_cpu(event):
    """Sum disjoint process CPU times, never child times or per-thread stat again."""
    values = defaultdict(float)
    for p in event.get("processes", []):
        v = p.get("cpu_percent_one_core")
        if number(v):
            values[p["group"]] += v
    return dict(values)


class ResourceHistory:
    def __init__(self, limit=1800):
        self.rows = {b: deque(maxlen=limit) for b in SOURCES.values()}
        self.last = {}

    def add(self, event, received):
        board = event["board"]
        key = (event["session"], event["seq"])
        if self.last.get(board) == key:
            return
        self.last[board] = key
        self.rows[board].append({"received": received, "cpu": event["cpu"]["total_percent"],
                                 "memory": event["memory"]["used_mib"],
                                 "gpu": event["gpu"]["load_percent"], "groups": group_cpu(event)})


def brief(event, age):
    if not event:
        return "미수신 · 계측 도구/보드 연결 확인"
    if age is None or age > STALE_S:
        return "수신 지연 · 마지막 값은 실시간 아님"
    if event["cpu"]["state"] != "live":
        return "첫 측정 구간 준비 중"
    cpu = event["cpu"]["total_percent"]
    ram = event["memory"].get("used_mib")
    text = f"CPU {cpu:.1f}% · RAM {ram/1024:.2f} GiB" if ram is not None else f"CPU {cpu:.1f}% · RAM 미제공"
    gpu = event["gpu"].get("load_percent")
    if gpu is not None:
        text += f" · GPU {gpu:.1f}%"
        peak = event["gpu"].get("max_percent")
        if peak is not None:
            text += f" (최대 {peak:.0f}%)"
    return text
