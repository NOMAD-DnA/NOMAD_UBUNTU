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
import tempfile
import threading
import uuid
import time
from typing import Callable

from control import run_remote, stop_boards, validate_host
from clock_sync import synchronize_clock

REMOTE = Path(__file__).resolve().parent / "remote"


def start_boards(
    pi_host: str,
    jetson_host: str,
    password: str,
    progress: Callable[[dict], None] | None = None,
    use_gnss: bool = True,
    semantic_model: str = "b2",
    cloud_source: str = "opencv",
    slope_mode: str | None = None,
    oak_cloud_hz: str = "1",
    mapper: str = "legacy",
    mapper2_configs: dict | None = None,
    terrain_configs: dict | None = None,
    pi4_host: str | None = None,
) -> dict:
    """Reset this trial's owned modules, then launch its existing pipeline.

    Both model/path preflights must pass before any process is stopped. A failed
    stage halts startup. Already-started processes may remain; the caller can use
    the same explicit Stop action for cleanup. Passwords are never returned.
    use_gnss=False starts no Pi5 GNSS module (run directory only); fusion then
    runs on VIO alone.
    """
    if not isinstance(password, str) or not password:
        raise ValueError("시험 시작에는 LAN 설정을 위한 보드 비밀번호가 필요합니다.")
    # 2026-11-12: settings chosen before Start (trial_settings.py).
    if semantic_model not in ("b2", "indoor", "fcn"):
        raise ValueError("알 수 없는 인지 모델입니다.")
    if cloud_source not in ("opencv", "device", "opencv3way"):
        raise ValueError("알 수 없는 OAK 점군 방식입니다.")
    if oak_cloud_hz not in ("1", "2", "3", "5"):
        raise ValueError("알 수 없는 OAK 점군 주기입니다.")
    if slope_mode not in (None, "sequential", "parallel"):
        raise ValueError("알 수 없는 경사 계산 방식입니다.")
    if mapper not in ("legacy", "mapper2", "terrain", "mapper3"):
        raise ValueError("알 수 없는 지도(매퍼) 선택입니다.")
    started_at = time.monotonic()
    stage_started = {}
    mapper_note = ""
    pi_host, jetson_host = validate_host(pi_host), validate_host(jetson_host)
    if pi_host.split("@", 1)[1] == jetson_host.split("@", 1)[1]:
        raise ValueError("Pi5와 Jetson 주소가 같습니다.")
    hosts = {"pi5": pi_host, "jetson": jetson_host}
    try:
        import board_address  # start/stop follow a board whose address changed
        board_address.register(pi5=pi_host, jetson=jetson_host)
    except Exception:
        pass
    stages: dict[str, dict] = {}
    output_lock = threading.Lock()

    def notify(stage: str, message: str, **extra) -> None:
        # 2026-11-12: elapsed seconds since Start, to see which stage is slow.
        now = time.monotonic()
        stage_started.setdefault(stage, now)
        if progress is not None:
            with output_lock:
                progress({"event": "progress", "stage": stage, "message": message,
                          "elapsed_s": round(now - started_at, 1), **extra})

    def timings() -> dict:
        order = sorted(stage_started.items(), key=lambda item: item[1])
        ends = [start for _stage, start in order[1:]] + [time.monotonic()]
        return {stage: round(end - start, 1) for (stage, start), end in zip(order, ends)}

    def result(ok: bool, stage: str, message: str) -> dict:
        return {
            "ok": ok,
            "state": "STARTED" if ok else "START_FAILED",
            "stage": stage,
            "message": message,
            "ready": False,
            "stages": stages,
            "timings_s": timings(),
            "total_s": round(time.monotonic() - started_at, 1),
        }

    def parallel(stage: str, jobs: dict[str, tuple[str, int, bool]], message: str) -> bool:
        notify(stage, message)
        found = {}
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = {
                pool.submit(run_remote, hosts[board], script if isinstance(script, Path) else REMOTE / script,
                            password, timeout_s=timeout, sudo=sudo): board
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
    # 2026-11-16: 모듈 세부 설정. A saved VIO override the Pi5 script cannot read is reported, not fatal.
    import module_params
    for note in module_params.start_notes(stages["preflight"]["pi5"].get("stdout", "")):
        notify("preflight", note, ok=False)

    notify("stop_previous", "이 실험의 기존 센서·VIO·융합·지도 종료")
    stopped = stop_boards(pi_host, jetson_host, password)
    stages["stop_previous"] = stopped
    if not stopped.get("ok"):
        return result(False, "stop_previous", "기존 실험 종료 실패. 새 모듈을 시작하지 않았습니다.")

    notify("clock", "센서 시작 전 보드 시간 동기화 확인 (Pi5 기준, Jetson 5ms 이내)")
    # Pi5 first: it may take this laptop's clock (no internet time), then Jetson steps to it.
    clocks = {"pi5": synchronize_clock(hosts["pi5"], password)}
    clocks["jetson"] = synchronize_clock(hosts["jetson"], password)
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

    # 2026-11-16 (user request): Wi-Fi power saving off on every board at every Start (this boot
    # only, remote/wifi_powersave_off.sh). Field logs showed planner VEHICLE_STATE_STALE with
    # wlan0 power management on. A failure is only reported; it never stops the Start.
    wifi_jobs = {"pi5": ("wifi_powersave_off.sh", 15, True), "jetson": ("wifi_powersave_off.sh", 15, True)}
    if pi4_host:
        try:
            pi4 = validate_host(pi4_host)
            if pi4.split("@", 1)[1] not in (pi_host.split("@", 1)[1], jetson_host.split("@", 1)[1]):
                hosts["pi4"] = pi4
                wifi_jobs["pi4"] = ("wifi_powersave_off.sh", 15, True)
        except ValueError:
            notify("wifi_power", "Pi4 주소가 올바르지 않아 Pi4 Wi-Fi 절전 끄기는 건너뜁니다.", ok=False)
    if not parallel("wifi_power", wifi_jobs, "보드 Wi-Fi 절전 끄기 (이번 부팅 동안)"):
        failed = [board for board, value in stages["wifi_power"].items() if not value.get("ok")]
        notify("wifi_power", "Wi-Fi 절전 끄기 실패: " + ", ".join(failed) + " · 시작은 계속합니다.", ok=False)

    if slope_mode is not None:
        # Saved on the Jetson before the map starts, so the map starts in this mode.
        notify("settings", "경사 계산 방식 저장 (지도 시작 전): " + slope_mode)
        from slope_control import switch_slope_mode
        slope = switch_slope_mode(jetson_host, slope_mode, password)
        stages["settings"] = {"slope": slope}
        notify("settings", "경사 계산: " + ("저장됨" if slope.get("ok") else "저장 실패 · 이전 방식 유지"),
               ok=bool(slope.get("ok")))

    if mapper == "mapper2":
        # 2026-11-13: new mapper settings go to ~/nomad_mapper2_ws/active_config before the map starts.
        import mapper2_settings
        notify("settings", "새 매퍼 설정 확인 (Jetson 저장값 사용 · 없으면 채움)")
        upload = tempfile.NamedTemporaryFile("w", suffix=".sh", prefix="nomad-mapper2-config-", delete=False)
        with upload:
            # 2026-11-13: values saved on the Jetson (settings dialog) win; only missing files are filled.
            upload.write(mapper2_settings.upload_script(mapper2_configs or mapper2_settings.defaults(), if_missing=True))
        try:
            uploaded = run_remote(jetson_host, Path(upload.name), password, timeout_s=20)
        finally:
            os.unlink(upload.name)
        stages.setdefault("settings", {})["mapper2"] = uploaded
        if not uploaded.get("ok"):
            return result(False, "settings", "새 매퍼 설정을 Jetson에 올리지 못했습니다. 센서는 시작하지 않았습니다.")
        notify("settings", "새 매퍼 설정 저장됨", ok=True)

    if mapper == "terrain":
        # 2026-11-13: improved legacy mapper settings (~/nomad_terrain_ws/active_config/terrain.json).
        import terrain_settings
        notify("settings", "기존 매퍼 개선 설정 확인 (Jetson 저장값 사용 · 없으면 채움)")
        upload = tempfile.NamedTemporaryFile("w", suffix=".sh", prefix="nomad-terrain-config-", delete=False)
        with upload:
            upload.write(terrain_settings.upload_script(terrain_configs or terrain_settings.defaults(), if_missing=True))
        try:
            uploaded = run_remote(jetson_host, Path(upload.name), password, timeout_s=20)
        finally:
            os.unlink(upload.name)
        stages.setdefault("settings", {})["terrain"] = uploaded
        if not uploaded.get("ok"):
            return result(False, "settings", "기존 매퍼 개선 설정을 Jetson에 올리지 못했습니다. 센서는 시작하지 않았습니다.")
        notify("settings", "기존 매퍼 개선 설정 저장됨", ok=True)

    pi_sensor_script = "start_pi_gnss.sh" if use_gnss else "start_pi_run_only.sh"
    # OAK cloud source is chosen at start (no switch afterwards).
    jetson_script = tempfile.NamedTemporaryFile("w", suffix=".sh", prefix="nomad-start-jetson-", delete=False)
    start_token = uuid.uuid4().hex
    with jetson_script:
        jetson_script.write("export NOMAD_CLOUD_SOURCE=" + cloud_source + "\n" +
                            "export NOMAD_SEMANTIC_MODEL=" + semantic_model + "\n" +
                            "export NOMAD_START_TOKEN=" + start_token + "\n" +
                            "export NOMAD_OAK_CLOUD_HZ=" + oak_cloud_hz + "\n" +
                            "export NOMAD_MAPPER=" + mapper + "\n" +
                            (REMOTE / "start_jetson.sh").read_text(encoding="utf-8"))
    # 2026-11-13: Pi5 VIO needs only the OAK stream, so it starts as soon as this Start's OAK is up
    # (or when the whole Jetson start has finished) instead of after every Jetson module.
    oak_check = tempfile.NamedTemporaryFile("w", suffix=".sh", prefix="nomad-oak-ready-", delete=False)
    with oak_check:
        oak_check.write("export NOMAD_START_TOKEN=" + start_token + "\n" +
                        (REMOTE / "check_jetson_oak_ready.sh").read_text(encoding="utf-8"))
    sensor_message = (("GNSS·" if use_gnss else "GNSS 제외 · ") + "카메라·LiDAR·인지·지도 기동 · 점군 " + cloud_source + " " +
                      oak_cloud_hz + "Hz" + {"mapper2": " · 새 매퍼", "terrain": " · 기존 매퍼 개선", "mapper3": " · 새 매퍼3 v3"}.get(mapper, "") +
                      " · 인지 " + semantic_model)

    def remote_job(board: str, script, timeout: int) -> dict:
        try:
            return run_remote(hosts[board], script if isinstance(script, Path) else REMOTE / script,
                              password, timeout_s=timeout, sudo=False)
        except Exception as exc:
            return {"ok": False, "reason": type(exc).__name__, "state": "FAILED", "stdout": "", "returncode": None}

    def report(stage: str, board: str, value: dict) -> None:
        notify(stage, f"{board}: {'완료' if value.get('ok') else '실패'}", board=board, ok=bool(value.get("ok")),
               reason=value.get("reason", ""),
               detail="\n".join(value.get("stdout", "").splitlines()[-8:]) if not value.get("ok") else "")

    try:
        notify("sensors", sensor_message)
        with ThreadPoolExecutor(max_workers=3) as pool:
            pi_future = pool.submit(remote_job, "pi5", pi_sensor_script, 25)
            jetson_future = pool.submit(remote_job, "jetson", Path(jetson_script.name), 90)
            pi_sensors = pi_future.result()
            report("sensors", "pi5", pi_sensors)
            localization_future = None
            if pi_sensors.get("ok"):
                deadline = time.monotonic() + 90
                while not jetson_future.done() and time.monotonic() < deadline:
                    ready = remote_job("jetson", Path(oak_check.name), 8)
                    if "OAK_READY" in ready.get("stdout", "").splitlines():
                        notify("localization", "OAK 영상 확인 → Pi5 Basalt VIO → 융합 순서로 기동 (Jetson 나머지 기동과 동시)")
                        localization_future = pool.submit(remote_job, "pi5", "start_pi_vio_fusion.sh", 40)
                        break
                    time.sleep(1.0)
            jetson_sensors = jetson_future.result()
            report("sensors", "jetson", jetson_sensors)
            # 2026-11-16: class-boundary px / initial-area values the Jetson actually used (terrain / mapper3 only).
            import mapper_settings
            mapper_note = mapper_settings.start_note(jetson_sensors.get("stdout", ""))
            if mapper_note:
                notify("sensors", mapper_note, ok=True)
            # 2026-11-16: '경사 점수 계수' actually used, or the "rebuild needed" warning (slope_k.py).
            import slope_k
            slope_note, slope_ok = slope_k.start_note(jetson_sensors.get("stdout", ""))
            if slope_note:
                notify("sensors", slope_note, ok=slope_ok)
            stages["sensors"] = {"pi5": pi_sensors, "jetson": jetson_sensors}
            if not (pi_sensors.get("ok") and jetson_sensors.get("ok")):
                if localization_future is not None:
                    stages["localization"] = {"pi5": localization_future.result()}
                return result(False, "sensors", "센서/인지 기동 실패. 일부 모듈이 남아 있을 수 있으니 종료 버튼으로 정리해 주세요.")
            if localization_future is None:  # OAK readiness not seen in time: start after the Jetson, as before
                notify("localization", "Pi5 Basalt VIO → 융합 순서로 기동")
                localization_future = pool.submit(remote_job, "pi5", "start_pi_vio_fusion.sh", 40)
            localization = localization_future.result()
            report("localization", "pi5", localization)
            stages["localization"] = {"pi5": localization}
            for note in module_params.start_notes(pi_sensors.get("stdout", ""), jetson_sensors.get("stdout", ""),
                                                  localization.get("stdout", "")):
                notify("localization", note, ok=True)
    finally:
        os.unlink(jetson_script.name)
        os.unlink(oak_check.name)
    if not stages["localization"]["pi5"].get("ok"):
        return result(False, "localization", "VIO/융합 기동 실패. 센서는 유지되며 종료 버튼으로 정리할 수 있습니다.")
    if not parallel("running", {
        "pi5": ("check_pi_started.sh", 18, False),
        "jetson": ("check_jetson_started.sh", 25, False),
    }, "기동한 프로세스 확인"):
        return result(False, "running", "일부 모듈이 종료되었습니다. 화면에서 실패 원인을 확인해 주세요.")
    model_note = ""
    if semantic_model == "fcn":
        # b2/indoor start directly in start_jetson.sh (2026-11-13); FCN still starts as b2 and switches once.
        notify("model", "인지 모델 전환: " + semantic_model)
        from model_control import switch_model
        switched = switch_model(jetson_host, semantic_model, password)
        stages["model"] = switched
        notify("model", "인지 모델 " + ("전환 완료" if switched.get("ok") else "전환 실패 · 기본 모델로 실행 중"),
               ok=bool(switched.get("ok")))
        if not switched.get("ok"):
            model_note = " 인지 모델 전환은 실패해 기본(SegFormer 야지)으로 실행 중입니다. 실시간 화면 설정에서 다시 적용하세요."
    notify("started", "기동 완료. 센서 수신·READY는 실시간 화면에서 확인합니다.", ok=True)
    return result(True, "started", "모듈 기동 완료. 실제 데이터·READY·지도 정확도는 실시간 화면에서 확인합니다." + model_note +
                  (" " + mapper_note if mapper_note else "") +
                  (" " + module_params.VIO_UNSUPPORTED + "." if "VIO_SETTINGS=unsupported" in
                   stages["preflight"]["pi5"].get("stdout", "") else ""))


def _mapper2_configs(mapper: str) -> dict | None:
    """The laptop's saved new-mapper settings (same state folder as the launcher)."""
    if mapper != "mapper2":
        return None
    import mapper2_settings
    state = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))).expanduser() / "nomad-trial"
    return mapper2_settings.load(state)


def _terrain_configs(mapper: str) -> dict | None:
    """The laptop's cached improved-legacy-mapper settings (only used when the Jetson has none)."""
    if mapper != "terrain":
        return None
    import terrain_settings
    state = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))).expanduser() / "nomad-trial"
    return terrain_settings.load(state)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-host", required=True, help="user@Pi5주소")
    parser.add_argument("--jetson-host", required=True, help="nvidia@Jetson주소")
    parser.add_argument("--no-gnss", action="store_true", help="Pi5 GNSS 모듈을 켜지 않음 (VIO만으로 위치)")
    parser.add_argument("--semantic-model", default="b2", choices=("b2", "indoor", "fcn"))
    parser.add_argument("--cloud-source", default="opencv", choices=("opencv", "device", "opencv3way"))
    parser.add_argument("--oak-cloud-hz", default="1", choices=("1", "2", "3", "5"))
    parser.add_argument("--slope-mode", default=None, choices=("sequential", "parallel"))
    parser.add_argument("--mapper", default="legacy", choices=("legacy", "mapper2", "terrain", "mapper3"))
    parser.add_argument("--pi4-host", default=None, help="user@Pi4주소 (Wi-Fi 절전 끄기만, 선택)")
    args = parser.parse_args(argv)
    password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")

    def emit(event: dict) -> None:
        print(json.dumps(event, ensure_ascii=False), flush=True)

    try:
        response = start_boards(args.pi_host, args.jetson_host, password, progress=emit,
                                use_gnss=not args.no_gnss, semantic_model=args.semantic_model,
                                cloud_source=args.cloud_source, slope_mode=args.slope_mode,
                                oak_cloud_hz=args.oak_cloud_hz,
                                mapper=args.mapper, mapper2_configs=_mapper2_configs(args.mapper),
                                terrain_configs=_terrain_configs(args.mapper), pi4_host=args.pi4_host)
    except ValueError as exc:
        response = {"ok": False, "state": "START_FAILED", "stage": "arguments", "message": str(exc), "ready": False}
    except Exception as exc:
        response = {"ok": False, "state": "START_FAILED", "stage": "unexpected", "message": type(exc).__name__, "ready": False}
    emit({"event": "result", **response})
    return 0 if response["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
