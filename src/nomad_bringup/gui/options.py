"""Typed options loaded from existing YAML; write overrides only for the current run."""

from copy import deepcopy
from pathlib import Path
import json
import math
import html
import re
import yaml
from option_help import brief_description, detailed_description
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QMessageBox,
    QScrollArea,
    QSpinBox,
    QTabWidget,
    QTextBrowser,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

ROOT = Path(__file__).resolve().parents[3]
FILES = {
    "terrain": ["src/nomad_perception/config/navigation_terrain.yaml"],
    "fusion": ["src/nomad_localization/config/fusion.yaml"],
    "planning": [
        "src/nomad_path_planning/config/planning.yaml",
        "src/nomad_perception/config/terrain_planning.yaml",
    ],
    "control": ["src/nomad_control/config/parameters.yaml"],
}
TITLES = {
    "terrain": "지도·주행가능성",
    "fusion": "GNSS·VIO 융합",
    "planning": "플래닝·후진 복구",
    "control": "차량 제어",
}
SECTIONS = {
    "all": list(FILES),
    "map": ["terrain"],
    "localization": ["fusion"],
    "planning": ["planning"],
    "control": ["control"],
}
LOCKED = {
    "use_sim_time",
    "frame_id",
    "local_frame_id",
    "body_frame_id",
    "auto_restart",
    "gpp_algorithm",
    "lpp_algorithm",
    "track",
}
LABELS = {
    'worker_timeout_wall_sec': '지도 계산 응답 제한 (초)',
    "resolution": "셀 크기 (m)",
    "size": "초기 지도 크기 (m)",
    "max_size": "최대 지도 크기 (m)",
    "rate": "지도 갱신 목표 (Hz)",
    "stride": "깊이 샘플 간격 (픽셀)",
    "min_confidence": "분할 최소 신뢰도",
    "ground_ids": "지면 클래스 ID",
    "slope_limit_deg": "차단 경사 기준 (°)",
    "local_max_age": "지역 관측 유지 (초)",
    "bootstrap_radius": "초기 전방 여유 반경 (m)",
    "bootstrap_override": "초기 출발 여유 우선 적용",
    "obstacle_cost_radius": "장애물 비용 반경 (m)",
    "passable_semantic_ids": "통행 허용 식생 클래스 ID",
    "window_s": "최적화 윈도 (초)",
    "keyframe_hz": "최적화 포즈 생성 (Hz)",
    "solve_hz": "최적화 실행 (Hz)",
    "max_iterations": "최적화 최대 반복",
    "solver_time_s": "최적화 시간 제한 (초)",
    "vio_translation_sigma_per_sqrt_s": "VIO 이동 불확실성",
    "vio_rotation_sigma_per_sqrt_s": "VIO 회전 불확실성",
    "gps_horizontal_sigma_m": "GNSS 수평 σ (m)",
    "gps_vertical_sigma_m": "GNSS 수직 σ (m)",
    "gps_nonfixed_sigma_m": "RTK 미고정 σ (m)",
    "use_reported_gps_covariance": "GNSS 보고 공분산 사용",
    "robust_loss_sigma": "강건 손실 기준 σ",
    "gps_max_speed_mps": "GNSS 속도 판정 한계 (m/s)",
    "gps_jump_margin_m": "GNSS 점프 여유 (m)",
    "gps_max_age_s": "GNSS 최대 지연 (초)",
    "gps_reacquire_s": "GNSS 재수용 확인 (초)",
    "gps_reacquire_agreement_m": "GNSS 재수용 일치 범위 (m)",
    "quality_max_age_s": "GNSS 품질 최대 지연 (초)",
    "vio_max_speed_mps": "VIO 속도 판정 한계 (m/s)",
    "vio_max_rotation_rate_rps": "VIO 회전 판정 한계 (rad/s)",
    "vio_jump_margin_m": "VIO 점프 여유 (m)",
    "vio_stale_s": "VIO 소실 판정 (초)",
    "restart_disagreement_m": "재초기화 불일치 기준 (m)",
    "restart_persistence_s": "실패 지속 판정 (초)",
    "restart_cooldown_s": "재초기화 대기 (초)",
    "max_restarts": "재초기화 횟수 한계",
    "alignment_require_rtk_fixed": "초기정렬에 RTK FIX 요구",
    "alignment_min_span_m": "초기정렬 최소 이동 (m)",
    "alignment_max_rmse_m": "초기정렬 허용 RMSE (m)",
    "path_max_points": "표시 궤적 최대 점 수",
    "datum": "GNSS 원점 [위도, 경도, 고도]",
    "antenna_in_imu": "IMU 기준 안테나 위치 [x,y,z] (m)",
    "gpp.time_budget": "전역 탐색 시간 제한 (초, 0=해제)",
    "gpp.proximity_radius": "GPP 장애물 근접 비용 반경",
    "gpp.proximity_weight": "GPP 장애물 근접 추가 가중치",
    "gpp.max_expansions": "전역 탐색 확장 한계",
    "gpp.weighted_astar_weight": "Weighted A* 가중치",
    "unknown_penalty": "미관측 영역 탐색 비용",
    "rpp.lookahead": "RPP 전방 추종 거리 (m)",
    "rpp.lateral_accel": "RPP 횡가속도 기준 (m/s²)",
    "forward_speed": "전진 목표 속도 (m/s, 차량 현재 제한 1)",
    "managed_recovery": "별도 후진 복구 사용",
    "allow_unknown": "미관측 영역 허용",
    "require_sensor_health": "센서 상태 확인",
    "sensor_health_timeout": "센서 상태 만료 (초)",
    "recovery.behind_seconds": "뒤쪽 경로 확인 (초)",
    "recovery.stuck_seconds": "정지 판정 시간 (초)",
    "recovery.stuck_radius": "정지 판정 범위 (m)",
    "recovery.min_retreat": "최소 후진 이력 (m)",
    "recovery.max_retreat": "최대 후진 거리 (m)",
    "recovery.exit_horizon": "탈출 탐색 길이 (m)",
    "recovery.exit_separation": "탈출 경로 분리 거리 (m)",
    "recovery.exit_speed": "탈출 전진 속도 (m/s)",
    "recovery.adjust_length": "자세 보정 후진 거리 (m)",
    "recovery.adjust_attempts": "자세 보정 시도 수",
    "recovery.exit_blocked_seconds": "탈출 차단 확인 (초)",
    "recovery.speed": "후진 복구 속도 (m/s)",
    "recovery.timeout": "복구 시간 한계 (초)",
    "recovery.max_attempts": "복구 시도 한계",
    "recovery.gate_ttl": "실패 접근 기억 (초)",
    "control_rate": "제어 주기 (Hz)",
    "command_wall_timeout": "명령 만료 정지 (실시간 초)",
    "health_label": "상태 표시 이름",
    "auto_restart": "자동 VIO 재시작 (현재 비활성 정책)",
}

SENSOR_LABELS = {
    "model_reference": "센서 모델 (참고)", "width": "영상 너비 (px)", "height": "영상 높이 (px)",
    "fps": "영상 주기 (Hz)", "hz": "출력 주기 (Hz)", "baseline_m": "양안 간격 (m)",
    "depth_min_m": "깊이 최소 (m)", "depth_max_m": "깊이 최대 (m)",
    "depth_noise_sigma_m": "깊이 노이즈 (현재 미적용)", "rgb_hfov_deg": "RGB 수평 화각 (°)",
    "rgb_vfov_deg": "RGB 수직 화각 (°)", "mono_hfov_deg": "양안 수평 화각 (°)",
    "mono_vfov_deg": "양안 수직 화각 (°)", "rgb_fx": "RGB fx (비우면 화각으로 계산)",
    "rgb_fy": "RGB fy (비우면 화각으로 계산)", "mono_fx": "양안 fx (비우면 화각으로 계산)",
    "mono_fy": "양안 fy (비우면 화각으로 계산)", "cx": "주점 cx (비우면 영상 중앙)",
    "cy": "주점 cy (비우면 영상 중앙)", "mount_x_m": "설치 X (m)",
    "mount_y_m": "설치 Y (m)", "mount_z_m": "설치 Z (m)", "mono_previews": "양안 미리보기 (현재 미적용)",
    "fov_deg": "시야각 (°)", "nrays": "스캔 점 수", "min_range_m": "최소 거리 (m)",
    "max_range_m": "최대 거리 (m)", "range_noise_sigma_m": "거리 노이즈 σ (m)",
    "angle_noise_sigma_deg": "각도 노이즈 (현재 미적용)", "yaw_deg": "Yaw (°)",
    "pitch_deg": "Pitch (°)", "roll_deg": "Roll (°)", "raytrace_3d": "3D 지형 스캔",
    "enabled": "GPS 사용", "visualization": "GPS 궤적 표시", "latitude_deg": "원점 위도 (°)",
    "longitude_deg": "원점 경도 (°)", "elevation_m": "원점 고도 (m)",
    "position_noise_horizontal_sigma_m": "수평 노이즈 σ (m)",
    "position_noise_vertical_sigma_m": "수직 노이즈 σ (m)",
}

# Physical dimensions are written explicitly; sigma is standard deviation, not variance.
UNITS = {
    "resolution": "m", "size": "m", "max_size": "m", "rate": "Hz", "fps": "Hz", "hz": "Hz",
    "stride": "px", "width": "px", "height": "px", "nrays": "points/scan",
    "min_confidence": "0–1", "ground_ids": "class ID", "passable_semantic_ids": "class ID",
    "slope_limit_deg": "deg", "local_max_age": "s, ROS time", "bootstrap_radius": "m",
    "obstacle_cost_radius": "m", "window_s": "s", "keyframe_hz": "Hz", "solve_hz": "Hz",
    "max_iterations": "iterations", "solver_time_s": "s, wall time",
    "vio_translation_sigma_per_sqrt_s": "m/√s", "vio_rotation_sigma_per_sqrt_s": "rad/√s",
    "gps_horizontal_sigma_m": "m", "gps_vertical_sigma_m": "m", "gps_nonfixed_sigma_m": "m",
    "robust_loss_sigma": "dimensionless", "gps_max_speed_mps": "m/s", "gps_jump_margin_m": "m",
    "gps_max_age_s": "s", "gps_reacquire_s": "s", "gps_reacquire_agreement_m": "m",
    "quality_max_age_s": "s", "vio_max_speed_mps": "m/s", "vio_max_rotation_rate_rps": "rad/s",
    "vio_jump_margin_m": "m", "vio_stale_s": "s", "restart_disagreement_m": "m",
    "restart_persistence_s": "s", "restart_cooldown_s": "s", "max_restarts": "attempts",
    "alignment_min_span_m": "m", "alignment_max_rmse_m": "m", "path_max_points": "points",
    "datum": "lat: deg, lon: deg, alt: m", "antenna_in_imu": "x,y,z: m",
    "gpp.time_budget": "s, wall time; 0=unlimited", "gpp.max_expansions": "expansions",
    "gpp.proximity_radius": "m", "gpp.proximity_weight": "dimensionless",
    "gpp.weighted_astar_weight": "dimensionless", "unknown_penalty": "dimensionless",
    "rpp.lookahead": "m", "rpp.lateral_accel": "m/s²", "max_steer": "rad", "forward_speed": "m/s",
    "sensor_health_timeout": "s", "recovery.behind_seconds": "s", "recovery.stuck_seconds": "s",
    "recovery.stuck_radius": "m", "recovery.min_retreat": "m", "recovery.max_retreat": "m",
    "recovery.exit_horizon": "m", "recovery.exit_separation": "m", "recovery.exit_speed": "m/s",
    "recovery.adjust_length": "m", "recovery.adjust_attempts": "attempts",
    "recovery.exit_blocked_seconds": "s", "recovery.speed": "m/s", "recovery.timeout": "s",
    "recovery.max_attempts": "attempts", "recovery.gate_ttl": "s", "control_rate": "Hz",
    "command_wall_timeout": "s, wall time", "track": "m",
    "baseline_m": "m", "depth_min_m": "m", "depth_max_m": "m", "depth_noise_sigma_m": "m",
    "rgb_hfov_deg": "deg", "rgb_vfov_deg": "deg", "mono_hfov_deg": "deg", "mono_vfov_deg": "deg",
    "rgb_fx": "px", "rgb_fy": "px", "mono_fx": "px", "mono_fy": "px", "cx": "px", "cy": "px",
    "mount_x_m": "m", "mount_y_m": "m", "mount_z_m": "m", "fov_deg": "deg",
    "min_range_m": "m", "max_range_m": "m", "range_noise_sigma_m": "m", "angle_noise_sigma_deg": "deg",
    "yaw_deg": "deg", "pitch_deg": "deg", "roll_deg": "deg", "latitude_deg": "deg",
    "longitude_deg": "deg", "elevation_m": "m", "position_noise_horizontal_sigma_m": "m",
    "position_noise_vertical_sigma_m": "m",
}
HELP = {
    "vio_translation_sigma_per_sqrt_s": "이동 오차 표준편차 계수. σₚ(Δt)=kₚ√Δt [m]; 구현상 Δt≥0.01 s.",
    "vio_rotation_sigma_per_sqrt_s": "회전 오차 표준편차 계수. σᵣ(Δt)=kᵣ√Δt [rad]; 구현상 Δt≥0.01 s.",
    "robust_loss_sigma": "σ로 정규화한 잔차 벡터의 크기에 적용하는 Huber 기준. 거리(m)나 공분산이 아닙니다.",
    "use_reported_gps_covariance": "NavSatFix 위치 공분산 단위는 m². 대각 원소의 √variance를 σ[m]로 변환하며 설정 σ를 하한으로 사용합니다.",
    "datum": "[latitude, longitude, altitude] = [deg, deg, m]. ENU 원점입니다.",
    "antenna_in_imu": "IMU 좌표계 기준 안테나 위치 [x,y,z] (m).",
    "min_confidence": "클래스 confidence, 0≤p≤1. 물리 단위 없음.",
    "track": "좌우 바퀴 간격 (m).",
}


SHORT_NAMES = {
    "vio_translation_sigma_per_sqrt_s": "VIO 이동 σ", "vio_rotation_sigma_per_sqrt_s": "VIO 회전 σ",
    "gps_horizontal_sigma_m": "GNSS 수평 σ", "gps_vertical_sigma_m": "GNSS 수직 σ",
    "gps_nonfixed_sigma_m": "RTK 미고정 σ", "robust_loss_sigma": "Huber 기준",
    "vio_topic": "VIO 토픽", "gps_topic": "GPS 토픽", "quality_topic": "품질 토픽",
    "frame_id": "전역 좌표계", "local_frame_id": "지역 좌표계", "body_frame_id": "기체 좌표계",
    "use_sim_time": "시뮬레이션 시간", "gpp_algorithm": "GPP", "lpp_algorithm": "LPP",
    "max_steer": "최대 조향각", "track": "좌우 바퀴 간격", "auto_restart": "VIO 자동 재시작",
    "rgb_fx": "RGB fx", "rgb_fy": "RGB fy", "mono_fx": "양안 fx", "mono_fy": "양안 fy",
    "cx": "주점 cx", "cy": "주점 cy", "datum": "GNSS 원점", "antenna_in_imu": "안테나 위치",
    "depth_noise_sigma_m": "깊이 노이즈 σ", "angle_noise_sigma_deg": "각도 노이즈 σ",
    "mono_previews": "양안 미리보기", "model_reference": "센서 모델",
}


def parameter_label(path, value, label=None):
    key = path[-1]
    if path[0] == "simulation":
        return {"map": "맵", "headless": "Gazebo 창 숨기기"}[key]
    name = SHORT_NAMES.get(key, label or LABELS.get(key, SENSOR_LABELS.get(key, key)))
    name = re.sub(r"\s*[（(].*?[）)]", "", name).strip()
    unit = UNITS.get(key, "")
    unit = unit.split(",")[0].split(";")[0]
    unit = {"dimensionless": "", "iterations": "회", "attempts": "회", "expansions": "회",
            "points": "개", "class ID": "", "lat: deg": "deg, deg, m", "x": "m"}.get(unit, unit)
    if key == "antenna_in_imu":
        unit = "m"
    if isinstance(value, bool):
        unit = ""
    return f"{name} [{unit}]" if unit else name

RANGES = {
    "gpp.proximity_weight": (0.0, 1000000.0),
    "resolution": (0.05, 1.0),
    "size": (10.0, 160.0),
    "max_size": (10.0, 320.0),
    "stride": (1, 32),
    "min_confidence": (0.0, 1.0),
    "slope_limit_deg": (1.0, 89.0),
    "rate": (0.2, 30.0),
    "local_max_age": (1.0, 600.0),
    "bootstrap_radius": (0.0, 10.0),
    "obstacle_cost_radius": (0.1, 5.0),
    "window_s": (0.5, 120.0),
    "keyframe_hz": (1.0, 50.0),
    "solve_hz": (0.1, 20.0),
    "max_iterations": (1, 1000),
    "solver_time_s": (0.001, 1.0),
    "control_rate": (1.0, 100.0),
    "forward_speed": (0.01, 1.0),
    "recovery.speed": (0.01, 1.0),
    "recovery.exit_speed": (0.01, 1.0),
}


def defaults():
    result = {
        "semantic": {"backend": "cuda", "rate": 10.0, "view": False, "model": ""},
        "configs": {},
        "visualization": {"rviz": True, "map_view": False, "initial_teleop": True},
        "simulation": {"map": "02_fork2_gentle", "headless": False, "vio": "basalt"},
        "sensors": yaml.safe_load((ROOT / "src/nomad_gazebo/config/sensors.yaml").read_text()),
    }
    for section, files in FILES.items():
        merged = {}
        for name in files:
            for node, block in yaml.safe_load((ROOT / name).read_text()).items():
                merged.setdefault(node, {"ros__parameters": {}})[
                    "ros__parameters"
                ].update(block["ros__parameters"])
        result["configs"][section] = merged
    return result


def overrides(menu_id, state, base, directory=None):
    args = []
    if menu_id in ("all", "localization", "map", "planning", "control"):
        from visualization_profile import rviz_profile
        rviz_config = rviz_profile(ROOT, menu_id, directory)
        show = str(state["visualization"]["rviz"]).lower()
        if menu_id == "all":
            args += [f"initial_teleop:={str(state['visualization']['initial_teleop']).lower()}", f"rviz_config:={rviz_config}", f"viewer:={str(state['visualization']['map_view']).lower()}"]
        else:
            args += [f"gui_rviz:={show}", f"gui_rviz_config:={rviz_config}"]
            if menu_id == "map":
                args += [f"gui_map_view:={str(state['visualization']['map_view']).lower()}"]
    topics = state["configs"]["fusion"]["fusion"]["ros__parameters"]
    default_topics = base["configs"]["fusion"]["fusion"]["ros__parameters"]
    topic_keys = ("vio_topic", "gps_topic", "quality_topic")
    if any(topics[key] != default_topics[key] for key in topic_keys):
        if menu_id in ("all", "localization"):
            args += [f"{key}:={topics[key]}" for key in topic_keys]
        if menu_id in ("all", "sim"):
            args.append("ros_remappings:=" + json.dumps({default_topics[key]: topics[key] for key in topic_keys}, separators=(",", ":")))
    if menu_id in ("all", "semantic"):
        for key, value in state["semantic"].items():
            if value == base["semantic"][key]:
                continue
            if key == "model" and value and not Path(value).is_file():
                raise ValueError("분할 모델 파일을 찾을 수 없습니다.")
            value = str(value).lower() if isinstance(value, bool) else str(value)
            if menu_id == "all":
                args.append(f"semantic_{key}:={value}")
            else:
                args.extend(["-p", f"{key}:={value}"])
    sensor_changed = state["sensors"] != base["sensors"]
    if sensor_changed and menu_id in ("all", "sim", "localization"):
        from sensor_profile import profile
        args.extend(profile(ROOT, menu_id, state, directory))
    for section in SECTIONS.get(menu_id, []):
        if menu_id == "all" and section == "control":
            continue
        if sensor_changed and section == "fusion" and menu_id in ("all", "localization"):
            continue
        data = state["configs"][section]
        changed = data != base["configs"][section]
        if menu_id == "all" and section == "planning":
            changed |= state["configs"]["control"] != base["configs"]["control"]
        if not changed:
            continue
        if section in ("planning", "control") and menu_id == "all":
            data = deepcopy(state["configs"]["planning"])
            data.update(state["configs"]["control"])
        filename = f"{section}.yaml"
        if directory is not None:
            path = Path(directory) / filename
            path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
        else:
            path = f"<이번 실행/{filename}>"
        if menu_id == "all":
            key = {
                "terrain": "terrain_config",
                "fusion": "fusion_config",
                "planning": "params_file",
                "control": "params_file",
            }[section]
            args.append(f"{key}:={path}")
        elif menu_id == "localization":
            args.append(f"config:={path}")
        elif menu_id in ("planning", "control"):
            args.append(f"params_file:={path}")
        else:
            args.extend(["--params-file", str(path)])
    return args


class OptionsDialog(QDialog):
    def __init__(self, parent, menu_id, state):
        super().__init__(parent)
        self.setWindowTitle("실행 옵션")
        self.resize(760, 680)
        self.menu_id = menu_id
        self.state = deepcopy(state)
        self.editors = {}
        self.detail_titles = {}
        self.option_labels = {}
        self.option_targets = {}
        self.selected_option = None
        layout = QVBoxLayout(self)
        note = QLabel(
            "옵션을 클릭하면 아래에 간단한 설명이 표시됩니다."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        tabs = QTabWidget()
        self.splitter = QSplitter()
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(tabs, 1)
        self.clicked_hint = QLabel()
        self.clicked_hint.setWordWrap(True)
        self.clicked_hint.setTextFormat(Qt.PlainText)
        self.clicked_hint.setStyleSheet("background:#eef3f2;padding:8px;border-radius:4px;")
        self.clicked_hint.hide()
        left_layout.addWidget(self.clicked_hint)
        self.splitter.addWidget(left_panel)
        self.details = QTextBrowser()
        self.details.setOpenExternalLinks(False)
        self.details.setMinimumWidth(360)
        self.details.hide()
        self.splitter.addWidget(self.details)
        layout.addWidget(self.splitter)
        if menu_id in ("all", "localization", "map", "planning", "control"):
            form = self.tab(tabs, "시각화")
            self.add(form, ("visualization", "rviz"), "RViz", state["visualization"]["rviz"])
            if menu_id == "all":
                self.add(form, ("visualization", "initial_teleop"), "초기 정렬 조종 창", state["visualization"]["initial_teleop"])
            if menu_id in ("all", "map"):
                self.add(form, ("visualization", "map_view"), "경사·클래스·주행가능성 창", state["visualization"]["map_view"])
        if menu_id in ("all", "semantic"):
            form = self.tab(tabs, "의미분할")
            for key, value in state["semantic"].items():
                self.add(
                    form,
                    ("semantic", key),
                    {
                        "backend": "실행 장치",
                        "rate": "최대 분할 Hz",
                        "view": "분할 영상 창",
                        "model": "모델 ONNX (비우면 기본 B0)",
                    }[key],
                    value,
                )
        for section in SECTIONS.get(menu_id, []):
            form = self.tab(tabs, TITLES[section])
            for node, block in state["configs"][section].items():
                label = QLabel(node)
                label.setStyleSheet("font-weight:600;margin-top:12px;")
                form.addRow(label)
                for key, value in block["ros__parameters"].items():
                    self.add(
                        form,
                        (section, node, key),
                        LABELS.get(key, key),
                        value,
                        key in LOCKED,
                    )
        if menu_id in ("all", "sim"):
            form = self.tab(tabs, "맵·시뮬레이션")
            self.add(form, ("simulation", "map"), "맵 선택", state["simulation"]["map"])
            self.add(form, ("simulation", "headless"), "Gazebo 창 숨기기", state["simulation"]["headless"])
            for section, values in state["sensors"].items():
                form = self.tab(tabs, {"camera": "카메라", "imu": "IMU", "lidar": "LiDAR", "gps": "GPS"}[section])
                if section == "gps":
                    form.addRow(QLabel("Gazebo 기본 GNSS 설정입니다. RTK·실패 이벤트 노드는 별도 실행입니다."))
                if section == "lidar":
                    form.addRow(QLabel("angle_noise는 현재 모델에서 미적용이며 수정할 수 없습니다."))
                for key, value in values.items():
                    self.add(form, ("sensors", section, key), SENSOR_LABELS.get(key, key), value,
                             key in ("model_reference", "angle_noise_sigma_deg", "raytrace_3d", "depth_noise_sigma_m", "mono_previews"))
        if tabs.count() == 0:
            form = self.tab(tabs, "실행 설정")
            form.addRow(QLabel("이 메뉴의 맵·표시 설정은 메인 화면에서 선택합니다."))
        self.populate_details()
        detail_row = QHBoxLayout()
        detail_row.addStretch()
        self.detail_toggle = QCheckBox("자세한 설명")
        self.detail_toggle.toggled.connect(self.toggle_details)
        detail_row.addWidget(self.detail_toggle)
        layout.addLayout(detail_row)
        footer = QLabel("변경한 설정은 이미 실행 중인 노드에는 적용되지 않습니다.")
        footer.setWordWrap(True)
        layout.addWidget(footer)
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok
            | QDialogButtonBox.Cancel
            | QDialogButtonBox.RestoreDefaults
        )
        buttons.button(QDialogButtonBox.Ok).setText("적용")
        buttons.button(QDialogButtonBox.Cancel).setText("취소")
        buttons.button(QDialogButtonBox.RestoreDefaults).setText("기본값 복원")
        buttons.accepted.connect(self.validate_accept)
        buttons.rejected.connect(self.reject)
        buttons.button(QDialogButtonBox.RestoreDefaults).clicked.connect(self.restore)
        layout.addWidget(buttons)

    def populate_details(self):
        path = self.selected_option
        if path is None:
            self.details.setHtml("<p>설명을 볼 옵션 이름이나 입력칸을 클릭하세요.</p>")
            return
        title = self.detail_titles[path]
        paragraphs = detailed_description(path).split("\n\n")
        text = ''.join('<p>' + html.escape(part).replace("\n", "<br>") + '</p>' for part in paragraphs)
        key = html.escape('.'.join(path))
        unit = html.escape(UNITS.get(path[-1], "해당 없음"))
        self.details.setHtml('<html><head><style>body {line-height:1.5;} h3 {color:#247565;} p {margin-top:8px;margin-bottom:12px;}</style></head><body>'
                             + f'<h3>{html.escape(title)}</h3><p><code>{key}</code><br>단위: {unit}</p>{text}</body></html>')

    def eventFilter(self, target, event):
        path = self.option_targets.get(target)
        if path is not None and event.type() == QEvent.MouseButtonPress:
            self.clicked_hint.setText(" ".join(self.editors[path][0].toolTip().split()))
            self.clicked_hint.show()
            if self.selected_option != path:
                self.selected_option = path
                for key, label in self.option_labels.items():
                    label.setStyleSheet("color:#247565;font-weight:600;" if key == path else "")
                if hasattr(self, "detail_toggle") and self.detail_toggle.isChecked():
                    self.populate_details()
        return super().eventFilter(target, event)

    def toggle_details(self, visible):
        self.details.setVisible(visible)
        self.resize(1240 if visible else 760, max(680, self.height()))
        if visible:
            self.populate_details()
            self.splitter.setSizes([620, 580])

    def tab(self, tabs, title):
        page = QWidget()
        form = QFormLayout(page)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        tabs.addTab(scroll, title)
        return form

    def add(self, form, path, label, value, locked=False):
        if path == ("simulation", "map"):
            widget = QComboBox()
            for choice_label, identifier in self.parent().sources["maps"]:
                widget.addItem(choice_label, identifier)
            widget.setCurrentIndex(widget.findData(value))
        elif path == ("semantic", "backend"):
            widget = QComboBox()
            widget.addItems(["cuda", "cpu"])
            widget.setCurrentText(value)
        elif isinstance(value, bool):
            widget = QCheckBox()
            widget.setChecked(value)
        elif isinstance(value, int):
            widget = QSpinBox()
            widget.setRange(0 if value == 0 else 1, 1000000)
            widget.setValue(value)
        elif isinstance(value, float):
            widget = QDoubleSpinBox()
            widget.setDecimals(5)
            widget.setRange(-1000000 if path[0] == "sensors" else (0 if value == 0 else 0.00001), 1000000)
            widget.setValue(value)
            widget.setSingleStep(0.1)
        else:
            widget = QLineEdit(
                json.dumps(value) if isinstance(value, list) else str(value)
            )
        if path[-1] in RANGES and isinstance(widget, (QSpinBox, QDoubleSpinBox)):
            widget.setRange(*RANGES[path[-1]])
        widget.setEnabled(not locked)
        detail = brief_description(path)
        if locked and path[-1] != "auto_restart":
            detail += "\n현재 값은 연결·모델 구성에 맞춰 고정되어 있습니다."
        widget.setToolTip(detail)
        row = widget
        if path == ("semantic", "model"):
            row = QWidget()
            box = QHBoxLayout(row)
            box.setContentsMargins(0, 0, 0, 0)
            box.addWidget(widget)
            pick = QPushButton("파일 선택")
            pick.clicked.connect(lambda: self.pick(widget))
            box.addWidget(pick)
        self.detail_titles[path] = parameter_label(path, value, label)
        title = QLabel(self.detail_titles[path])
        title.setToolTip(widget.toolTip())
        title.setCursor(Qt.PointingHandCursor)
        self.option_labels[path] = title
        row.setToolTip(widget.toolTip())
        form.addRow(title, row)
        self.editors[path] = (widget, value, locked)
        for target in {title, row, widget, *row.findChildren(QWidget)}:
            self.option_targets[target] = path
            target.installEventFilter(self)

    def validate_accept(self):
        try:
            self.result_state = self.collect()
        except (ValueError, json.JSONDecodeError) as error:
            QMessageBox.warning(self, "설정 확인", str(error))
            return
        self.accept()

    def pick(self, widget):
        path, _ = QFileDialog.getOpenFileName(
            self, "ONNX 모델 선택", str(ROOT), "ONNX (*.onnx)"
        )
        if path:
            widget.setText(path)

    def restore(self):
        base = defaults()
        for path, (widget, original, locked) in self.editors.items():
            value = (
                base[path[0]][path[1]] if path[0] in ("simulation", "visualization") else
                base["sensors"][path[1]][path[2]] if path[0] == "sensors" else
                base["semantic"][path[1]]
                if path[0] == "semantic"
                else base["configs"][path[0]][path[1]]["ros__parameters"][path[2]]
            )
            if isinstance(widget, QCheckBox):
                widget.setChecked(value)
            elif isinstance(widget, QComboBox):
                widget.setCurrentIndex(widget.findData(value)) if path == ("simulation", "map") else widget.setCurrentText(value)
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                widget.setValue(value)
            else:
                widget.setText(
                    json.dumps(value) if isinstance(value, list) else str(value)
                )

    def collect(self):
        result = deepcopy(self.state)
        for path, (widget, original, locked) in self.editors.items():
            if locked:
                continue
            if isinstance(widget, QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, QComboBox):
                value = widget.currentData() if path == ("simulation", "map") else widget.currentText()
            elif isinstance(widget, (QSpinBox, QDoubleSpinBox)):
                value = widget.value()
            else:
                value = widget.text()
                if original is None:
                    value = None if value.strip() in ("", "None", "자동") else float(value)
                if isinstance(original, list):
                    value = json.loads(value)
                    if (
                        not isinstance(value, list)
                        or len(value) != len(original)
                        and path[-1] in ("datum", "antenna_in_imu")
                    ):
                        raise ValueError("벡터 길이를 유지해 주세요.")
                    if not all(
                        isinstance(v, (int, float))
                        and not isinstance(v, bool)
                        and math.isfinite(v)
                        for v in value
                    ):
                        raise ValueError("목록에는 숫자만 입력하세요.")
            if path[-1] in ("ground_ids", "passable_semantic_ids") and not all(
                isinstance(v, int) and 0 <= v < 10 for v in value
            ):
                raise ValueError("클래스 ID는 0~9 정수입니다.")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError("숫자는 유한한 값이어야 합니다.")
            if path[0] in ("simulation", "visualization"):
                result[path[0]][path[1]] = value
            elif path[0] == "sensors":
                result["sensors"][path[1]][path[2]] = value
            elif path[0] == "semantic":
                result["semantic"][path[1]] = value
            else:
                result["configs"][path[0]][path[1]]["ros__parameters"][path[2]] = value
        terrain = result["configs"]["terrain"]["live_terrain"]["ros__parameters"]
        if terrain["max_size"] < terrain["size"]:
            raise ValueError("최대 지도 크기는 초기 크기 이상이어야 합니다.")
        recovery = result["configs"]["planning"]["nomad_recovery_supervisor"][
            "ros__parameters"
        ]
        if recovery["recovery.max_retreat"] < recovery["recovery.min_retreat"]:
            raise ValueError("최대 후진 거리는 최소 이력 이상이어야 합니다.")
        topics = result["configs"]["fusion"]["fusion"]["ros__parameters"]
        names = [topics[key] for key in ("vio_topic", "gps_topic", "quality_topic")]
        if any(not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*", name) for name in names):
            raise ValueError("토픽은 /로 시작하는 ROS 이름이어야 합니다. 예: /localization/vio")
        if len(set(names)) != len(names):
            raise ValueError("VIO·GPS·quality는 서로 다른 토픽을 사용하세요.")
        from sensor_profile import validate
        validate(ROOT, result["sensors"])
        return result
