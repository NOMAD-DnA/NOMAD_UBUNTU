#!/usr/bin/env python3
"""Laptop-only startup of the existing 2026-11-06 manual mapping trial.

Remote code is shell glue for installed native modules. No installs, builds,
board power-off, planning or motor commands are performed. STARTED means the
processes were launched; sensor/localization READY is determined by the viewer.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import sys
import threading
from typing import Callable

from control import run_remote, stop_boards, validate_host
from clock_sync import synchronize_clock

REMOTE = Path(__file__).resolve().parent / "remote"


def start_boards(
    pi_host: str,
    jetson_host: str,
    password: str,
    progress: Callable[[dict], None] | None = None,
) -> dict:
    """Reset this trial's owned modules, then launch its existing pipeline.

    Both model/path preflights must pass before any process is stopped. A failed
    stage halts startup. Already-started processes may remain; the caller can use
    the same explicit Stop action for cleanup. Passwords are never returned.
    """
    if not isinstance(password, str) or not password:
        raise ValueError("시험 시작에는 LAN 설정을 위한 보드 비밀번호가 필요합니다.")
    pi_host, jetson_host = validate_host(pi_host), validate_host(jetson_host)
    if pi_host.split("@", 1)[1] == jetson_host.split("@", 1)[1]:
        raise ValueError("Pi5와 Jetson 주소가 같습니다.")
    hosts = {"pi5": pi_host, "jetson": jetson_host}
    stages: dict[str, dict] = {}
    output_lock = threading.Lock()

    def notify(stage: str, message: str, **extra) -> None:
        if progress is not None:
            with output_lock:
                progress({"event": "progress", "stage": stage, "message": message, **extra})

    def result(ok: bool, stage: str, message: str) -> dict:
        return {
            "ok": ok,
            "state": "STARTED" if ok else "START_FAILED",
            "stage": stage,
            "message": message,
            "ready": False,
            "stages": stages,
        }

    def parallel(stage: str, jobs: dict[str, tuple[str, int, bool]], message: str) -> bool:
        notify(stage, message)
        found = {}
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = {
                pool.submit(run_remote, hosts[board], REMOTE / script, password,
                            timeout_s=timeout, sudo=sudo): board
                for board, (script, timeout, sudo) in jobs.items()
            }
            for future in as_completed(futures):
                board = futures[future]
                try:
                    value = future.result()
                except Exception as exc:
                    # Do not expose credentials or a full SSH invocation.
                    value = {"ok": False, "reason": type(exc).__name__,
                             "state": "FAILED", "stdout": "", "returncode": None}
                found[board] = value
                notify(stage, f"{board}: {'완료' if value.get('ok') else '실패'}",
                       board=board, ok=bool(value.get("ok")), reason=value.get("reason", ""),
                       detail="\n".join(value.get("stdout", "").splitlines()[-8:])
                       if not value.get("ok") else "")
        stages[stage] = found
        return all(value.get("ok") for value in found.values())

    if not parallel("preflight", {
        "pi5": ("preflight_pi.sh", 18, False),
        "jetson": ("preflight_jetson.sh", 18, False),
    }, "보드 모델·설치된 작업 폴더 확인"):
        return result(False, "preflight", "보드 확인 실패. 실행 중인 모듈은 변경하지 않았습니다.")
    for board in hosts:
        if f"ROLE={board}\n" not in stages["preflight"][board].get("stdout", ""):
            stages["preflight"][board]["ok"] = False
            stages["preflight"][board]["reason"] = "role_marker_missing"
            return result(False, "preflight", f"{board} 역할 확인 실패. 모듈은 변경하지 않았습니다.")

    notify("stop_previous", "이 실험의 기존 센서·VIO·융합·지도 종료")
    stopped = stop_boards(pi_host, jetson_host, password)
    stages["stop_previous"] = stopped
    if not stopped.get("ok"):
        return result(False, "stop_previous", "기존 실험 종료 실패. 새 모듈을 시작하지 않았습니다.")

    notify("clock", "센서 시작 전 보드 시간 동기화 확인 (Pi5 기준, Jetson 5ms 이내)")
    with ThreadPoolExecutor(max_workers=2) as pool:
        clock_jobs = {board: pool.submit(synchronize_clock, host, password)
                      for board, host in hosts.items()}
        clocks = {board: job.result() for board, job in clock_jobs.items()}
    stages["clock"] = clocks
    if not all(value.get("ok") for value in clocks.values()):
        reasons = " / ".join(f"{board}: {value.get('message', value.get('reason', ''))}" for board, value in clocks.items() if not value.get("ok"))
        return result(False, "clock", "보드 시간 동기화 확인 실패. " + reasons + ". 센서는 시작하지 않았습니다.")

    if not parallel("network", {
        "pi5": ("restore_network.sh", 40, True),
        "jetson": ("restore_network.sh", 40, True),
    }, "직결 LAN·임시 수신 버퍼 복구"):
        return result(False, "network", "LAN 설정 실패. 센서 시작을 중단했습니다.")
    if not parallel("lan_check", {"pi5": ("check_lan_pi.sh", 12, False)},
                    "Pi5 ↔ Jetson 직결 LAN 연결 확인"):
        return result(False, "lan_check", "직결 LAN 통신 실패. 케이블 연결을 확인해 주세요.")

    if not parallel("sensors", {
        "pi5": ("start_pi_gnss.sh", 25, False),
        "jetson": ("start_jetson.sh", 90, False),
    }, "GNSS·카메라·LiDAR·인지·지도 기동"):
        return result(False, "sensors", "센서/인지 기동 실패. 일부 모듈이 남아 있을 수 있으니 종료 버튼으로 정리해 주세요.")
    if not parallel("localization", {"pi5": ("start_pi_vio_fusion.sh", 40, False)},
                    "Pi5 Basalt VIO → 융합 순서로 기동"):
        return result(False, "localization", "VIO/융합 기동 실패. 센서는 유지되며 종료 버튼으로 정리할 수 있습니다.")
    if not parallel("running", {
        "pi5": ("check_pi_started.sh", 18, False),
        "jetson": ("check_jetson_started.sh", 25, False),
    }, "기동한 프로세스 확인"):
        return result(False, "running", "일부 모듈이 종료되었습니다. 화면에서 실패 원인을 확인해 주세요.")
    notify("started", "기동 완료. 센서 수신·READY는 실시간 화면에서 확인합니다.", ok=True)
    return result(True, "started", "모듈 기동 완료. 실제 데이터·READY·지도 정확도는 실시간 화면에서 확인합니다.")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-host", required=True, help="user@Pi5주소")
    parser.add_argument("--jetson-host", required=True, help="nvidia@Jetson주소")
    args = parser.parse_args(argv)
    password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")

    def emit(event: dict) -> None:
        print(json.dumps(event, ensure_ascii=False), flush=True)

    try:
        response = start_boards(args.pi_host, args.jetson_host, password, progress=emit)
    except ValueError as exc:
        response = {"ok": False, "state": "START_FAILED", "stage": "arguments", "message": str(exc), "ready": False}
    except Exception as exc:
        response = {"ok": False, "state": "START_FAILED", "stage": "unexpected", "message": type(exc).__name__, "ready": False}
    emit({"event": "result", **response})
    return 0 if response["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
