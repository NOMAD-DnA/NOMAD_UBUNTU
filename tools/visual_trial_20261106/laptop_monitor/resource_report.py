#!/usr/bin/env python3
"""Export resource samples from one existing .nomad file; never contact boards."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from resource_metrics import SOURCES, validate_resource
from session_recording import SessionReader


def percentile(values, p):
    values = sorted(values)
    if not values:
        return None
    pos = (len(values)-1)*p; lo = math.floor(pos); hi = math.ceil(pos)
    return values[lo]+(values[hi]-values[lo])*(pos-lo)


def export(recording, destination=None):
    recording = Path(recording).expanduser().resolve(strict=True)
    if destination is None:
        destination = recording.with_name(recording.stem+"_resources")
        n = 1
        while destination.exists():
            destination = recording.with_name(recording.stem+f"_resources_{n}"); n += 1
    destination = Path(destination).expanduser().absolute()
    destination.mkdir(parents=True, exist_ok=False)
    series = {b: [] for b in SOURCES.values()}
    counts = {"invalid": 0, "stale_or_baseline": 0, "duplicate": 0}
    last = {}; gaps = {b: 0 for b in series}; seen = set()
    board_columns = ["offset_s", "board", "t", "sample_monotonic_s", "session", "seq", "boot_id",
                     "online_cores", "cpu_percent", "interval_s", "ram_used_mib", "ram_available_mib",
                     "swap_used_mib", "gpu_percent", "gpu_frequency_mhz", "scan_ms",
                     "processes_total", "processes_omitted", "proc_read_failures"]
    proc_columns = ["offset_s", "board", "session", "seq", "pid", "ppid", "start_ticks", "name", "group",
                    "cpu_percent_one_core", "rss_mib", "threads"]
    core_columns = ["offset_s", "board", "session", "seq", "id", "percent", "frequency_mhz"]
    reader = SessionReader(recording)
    environment = {}
    try:
        with (destination/"board.csv").open("w", newline="") as bf, (destination/"processes.csv").open("w", newline="") as pf, \
                (destination/"cores.csv").open("w", newline="") as cf, (destination/"resources.jsonl").open("w") as raw:
            bw = csv.DictWriter(bf, fieldnames=board_columns); pw = csv.DictWriter(pf, fieldnames=proc_columns)
            cw = csv.DictWriter(cf, fieldnames=core_columns)
            bw.writeheader(); pw.writeheader(); cw.writeheader()
            for item in reader.iter_until(reader.duration):
                e = item["event"]
                if e.get("source") not in SOURCES:
                    continue
                try:
                    validate_resource(e, item["board"])
                except ValueError:
                    counts["invalid"] += 1; continue
                board = e["board"]; key = (board, e["boot_id"], e["session"], e["seq"])
                if key in seen:
                    counts["duplicate"] += 1; continue
                seen.add(key)
                raw.write(json.dumps(item, ensure_ascii=False, allow_nan=False)+"\n")
                environment[board] = {k: e[k] for k in ("model", "kernel", "boot_id")}
                # An offset-zero cached snapshot is not a new full interval inside this recording.
                if (item["offset_s"] <= 0 or e.get("_record_receive_age_s", 0) > 3 or e["cpu"]["state"] != "live"):
                    counts["stale_or_baseline"] += 1; continue
                previous = last.get(board)
                if previous and previous[0] == e["session"]:
                    gaps[board] += max(0, e["seq"]-previous[1]-1)
                last[board] = (e["session"], e["seq"])
                c=e["cpu"]; m=e["memory"]; g=e["gpu"]
                row = dict(offset_s=item["offset_s"], board=board,
                           **{k: e[k] for k in ("t", "sample_monotonic_s", "session", "seq", "boot_id")},
                           online_cores=c["online_cores"], cpu_percent=c["total_percent"], interval_s=c["interval_s"],
                           ram_used_mib=m["used_mib"], ram_available_mib=m["available_mib"], swap_used_mib=m["swap_used_mib"],
                           gpu_percent=g["load_percent"], gpu_frequency_mhz=g["frequency_mhz"],
                           **{k:e[k] for k in ("scan_ms", "processes_total", "processes_omitted", "proc_read_failures")})
                bw.writerow(row); series[board].append(row)
                for p in e["processes"]:
                    pw.writerow(dict(offset_s=item["offset_s"],board=board,session=e["session"],seq=e["seq"],**p))
                for core in c["cores"]:
                    cw.writerow(dict(offset_s=item["offset_s"],board=board,session=e["session"],seq=e["seq"],**core))
        summary = {"recording": str(recording), "recording_complete": reader.metadata.get("_recording",{}).get("complete"),
                   "duration_s": reader.duration, "environment": environment, "excluded": counts, "boards": {},
                   "gui_source_sha256": reader.metadata.get("gui_source_sha256"),
                   "limitations": ["1-second interval averages; sample maximum is not an instantaneous peak.",
                                   "Process list is bounded: known modules plus busy processes, not every process.",
                                   "RSS can share pages; do not sum RSS as physical RAM.",
                                   "CPU percentages on different boards are not equivalent compute capacity.",
                                   "No loop deadline or physical driving safety guarantee from CPU alone.",
                                   "Offsets are laptop receipt time, not synchronized board execution time."]}
        for board, rows in series.items():
            values = [r["cpu_percent"] for r in rows]; duration = sum(r["interval_s"] for r in rows)
            summary["boards"][board] = {"samples":len(rows), "missing_sequences":gaps[board],
                "sampled_interval_s":duration,
                "cpu_time_weighted_mean_percent":sum(r["cpu_percent"]*r["interval_s"] for r in rows)/duration if duration else None,
                "cpu_p95_percent":percentile(values,.95), "cpu_sample_max_percent":max(values) if values else None,
                "sampled_time_cpu_ge_90_s":sum(r["interval_s"] for r in rows if r["cpu_percent"] >= 90),
                "ram_sample_max_mib":max((r["ram_used_mib"] for r in rows if r["ram_used_mib"] is not None),default=None),
                "sampler_scan_p95_ms":percentile([r["scan_ms"] for r in rows],.95)}
        (destination/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
        fig, axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        for board, rows in series.items():
            if not rows: continue
            xs=[];cpu=[];ram=[];gpu=[];last_row=None
            for r in rows:
                if last_row is not None and (r["offset_s"]-last_row["offset_s"] > 3 or r["session"] != last_row["session"]):
                    xs.append(r["offset_s"]);cpu.append(math.nan);ram.append(math.nan);gpu.append(math.nan)
                xs.append(r["offset_s"]);cpu.append(r["cpu_percent"]);ram.append(r["ram_used_mib"]);gpu.append(r["gpu_percent"])
                last_row=r
            name="pi5" if board=="pi" else board
            axes[0].plot(xs,cpu,label=name);axes[1].plot(xs,ram,label=name);axes[2].plot(xs,gpu,label=name)
        for ax,label in zip(axes,("Board CPU (%)","RAM used (MiB)","GPU load (%)")):
            ax.set_ylabel(label);ax.grid(alpha=.2)
            if ax.lines: ax.legend()
        axes[0].set_ylim(0,100);axes[2].set_ylim(0,100);axes[-1].set_xlabel("Laptop receipt offset (s)")
        fig.tight_layout();fig.savefig(destination/"resources.png",dpi=150);plt.close(fig)
        lines=["# 보드 자원 사용량", "", f"기록: `{recording.name}`", "", "| 보드 | 표본 수 | 시간가중 평균 CPU | P95 CPU | 표본 최대 CPU |", "|---|---:|---:|---:|---:|"]
        for board,s in summary["boards"].items():
            fmt=lambda x:"미수신" if x is None else f"{x:.2f}%"
            lines.append(f'| {board} | {s["samples"]} | {fmt(s["cpu_time_weighted_mean_percent"])} | {fmt(s["cpu_p95_percent"])} | {fmt(s["cpu_sample_max_percent"])} |')
        lines.extend(["", "CPU는 1초 구간 평균이며 짧은 순간 피크를 보장하지 않는다. 보드별 CPU 백분율을 합산해 젯슨 이식 가능 여부를 확정할 수 없다.",
                      "GPP·LPP가 한 프로세스이면 플래닝 합산 부하다. RSS 합은 공유 메모리를 중복 계산할 수 있다.",
                      "원본 수치: board.csv / cores.csv / processes.csv / resources.jsonl. 미수신 값은 0으로 보간하지 않는다.",
                      "노트북 수신 시각 기준 그래프이며 보드 실행 시각의 정밀 동기 비교는 아니다.", "", "![자원 사용량](resources.png)"])
        (destination/"README.md").write_text("\n".join(lines)+"\n")
    finally:
        reader.close()
    return destination


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording");parser.add_argument("--output")
    args=parser.parse_args()
    print(export(args.recording,args.output))
