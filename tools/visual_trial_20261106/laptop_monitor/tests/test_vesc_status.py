"""2026-11-16 모터·서보(VESC) 표시 계약. 가짜 상태값만 쓰며 보드·출력에 접근하지 않는다."""
import os
from pathlib import Path
import sys
import time
import unittest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR', '/tmp/nomad_vesc_status_test')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt5 import QtWidgets
import vesc_status
from developer_control import ControlWindow
from drive_record_dialog import DriveRecordDialog

# 제어기 status_json 모양(가짜 값). 2026-11-16 키 포함.
STATUS = dict(state='IDLE', connected=True, telemetry_fresh=True, raw_erpm=1000.0, current_a=12.3, voltage_v=16.42,
              fault_code=0, fault_name='NONE', vesc_fault=False, esc_temp_c=31.2, esc_temp_state='ok', erpm_valid=True,
              stop_output='none', stop_output_a=0.0, stop_hold_basis='none', commanded_servo_position=0.512,
              requested_servo_position=0.53, commanded_motor_erpm=980.0, target_motor_erpm=1000.0, driver_ready=True,
              input_current_a=1.5, input_power_w=24.6, iq_a=11.0, id_a=0.1, duty=0.091, vd_v=-0.2, vq_v=1.4,
              esc_mos_temps_c=[30.1, 31.2, 30.8], charge_drawn_ah=0.125, charge_regen_ah=0.01, energy_drawn_wh=2.0,
              energy_regen_wh=0.2, tacho=1234, tacho_abs=4321, vesc_controller_id=111, telemetry_age_s=0.012,
              estimated_speed_mps=0.2687, estimated_speed_verified=False)
OLD_STATUS = {key: STATUS[key] for key in ('state', 'raw_erpm', 'current_a', 'voltage_v', 'fault_code', 'esc_temp_c',
                                          'esc_temp_state', 'erpm_valid', 'commanded_servo_position', 'driver_ready')}


def rows(data, fresh=True):
    return {title: (text, level) for title, text, level in vesc_status.status_rows(data, fresh)}


def controller_line(stamp, payload):
    return f'[controller] [INFO] [{stamp:.9f}] [nomad_vesc_trial]: {payload}'


VESC_TRACE = ('vesc_trace voltage_v=16.42 input_current_a=1.50 power_w=24.6 iq_a=11.00 duty=0.091 fault_code=0 '
              'fault=NONE servo_cmd=0.512 servo_target=0.530 hold_basis=load telemetry_age_ms=12')
CONTROL_TRACE = ('control_trace source=LPP phase=RUNNING requested_erpm=930.5 target_erpm=1000.0 output_erpm=1000.0 '
                 'measured_erpm=987.0 current_a=12.30 esc_temp_c=31.2 thermal=ok stop=none stop_a=0.0 erpm_valid=1 '
                 'start_gate=measured speed tracking reset_required=0')


class VescStatusRowsTest(unittest.TestCase):
    def test_new_controller_values_are_shown_with_units_and_caveats(self):
        shown = rows(STATUS)
        self.assertIn('16.42 V', shown['배터리'][0])
        self.assertIn('1.50 A(VESC 추정)', shown['배터리'][0])
        self.assertIn('iq 11.0 A', shown['모터 전류'][0])
        self.assertIn('0.27 m/s (환산 미검증)', shown['바퀴 속도'][0])
        self.assertEqual(shown['VESC 고장'], ('0 NONE (없음)', 'ok'))
        self.assertIn('실측 조향 아님', shown['서보'][0])
        self.assertIn('12 ms 전', shown['VESC 상태값'][0])
        self.assertTrue(vesc_status.controller_sends_vesc_values(STATUS))

    def test_old_controller_values_are_not_invented(self):
        shown = rows(OLD_STATUS)
        self.assertFalse(vesc_status.controller_sends_vesc_values(OLD_STATUS))
        self.assertIn('입력 — A', shown['배터리'][0])
        self.assertIn('iq — A', shown['모터 전류'][0])
        self.assertEqual(shown['VESC 고장'][0], '0 NONE (없음)')  # 이름은 펌웨어 5.03 표로 채움
        self.assertNotIn('m/s', shown['바퀴 속도'][0])
        self.assertTrue(all(text == '—' or '—' in text for _title, text in vesc_status.diagnostic_rows(OLD_STATUS)))

    def test_warning_levels(self):
        held = rows(dict(STATUS, erpm_valid=False, raw_erpm=0.0))
        self.assertEqual(held['바퀴 속도'][1], 'unknown')
        self.assertIn('측정값 아님', held['바퀴 속도'][0])
        fault = rows(dict(STATUS, fault_code=2, fault_name='UNDER_VOLTAGE', vesc_fault=True))
        self.assertEqual(fault['VESC 고장'], ('2 UNDER_VOLTAGE (저전압) · 제어기 잠금', 'fail'))
        self.assertEqual(rows(dict(STATUS, duty=-0.93))['듀티'], ('-0.930 · 전압 한계 근처', 'warn'))
        self.assertEqual(rows(dict(STATUS, esc_temp_state='stop'))['ESC 온도'][1], 'fail')
        self.assertEqual(rows(dict(STATUS, telemetry_age_s=0.5))['VESC 상태값'][1], 'warn')
        self.assertEqual(rows(STATUS, fresh=False)['VESC 상태값'][1], 'warn')
        self.assertEqual(vesc_status.fault_text(-1), '-1 NO_TELEMETRY (상태값 없음)')
        self.assertEqual(vesc_status.fault_text(40), '40 UNKNOWN')

    def test_vesc_trace_line_parses_without_inheriting_old_values(self):
        trace = vesc_status.vesc_trace_from_line(controller_line(100.5, VESC_TRACE))
        self.assertEqual(trace['voltage_v'], 16.42)
        self.assertEqual(trace['servo_cmd'], 0.512)
        self.assertEqual(trace['fault'], 'NONE')
        self.assertEqual(trace['hold_basis'], 'load')
        self.assertEqual(trace['stamp'], 100.5)
        partial = vesc_status.vesc_trace_from_line(controller_line(101., 'vesc_trace voltage_v=16.40 telemetry_age_ms=-1'))
        self.assertIsNone(partial['servo_cmd'])
        self.assertIsNone(partial['telemetry_age_ms'])
        self.assertIsNone(vesc_status.vesc_trace_from_line('[planner] vesc_trace voltage_v=1'))
        self.assertIsNone(vesc_status.vesc_trace_from_line(controller_line(1., CONTROL_TRACE)))


LINK_TRACE = 'link_trace n=20 gap_ms=62 age_min_ms=3 age_max_ms=15 expired=0 tick_max_ms=22'
LINK_BAD = 'link_trace n=12 gap_ms=410 age_min_ms=4 age_max_ms=380 expired=1 tick_max_ms=21'


class HoldAndLinkRowsTest(unittest.TestCase):
    """2026-11-17: 정지 유지 시간·발열 추정·오래 유지 경고, 제어 주기·LPP 줄 (제어기 f6f4946 이후)."""

    def test_keyboard_hold_row_uses_controller_estimates(self):
        held = rows(dict(STATUS, stop_output='hold', stop_output_a=30.0, stop_hold_basis='unsettled', stop_hold_s=12.4,
                         stop_hold_heat_w=15.12, stop_hold_long=True))
        self.assertEqual(held['정지 출력'][1], 'warn')
        self.assertIn('12초째', held['정지 출력'][0])
        self.assertIn('모터 발열 약 15 W(추정)', held['정지 출력'][0])
        self.assertIn('모터 온도 미측정', held['정지 출력'][0])
        short = rows(dict(STATUS, stop_output='hold', stop_output_a=12.5, stop_hold_basis='load', stop_hold_s=3.0,
                          stop_hold_heat_w=2.625, stop_hold_long=False))
        self.assertEqual(short['정지 출력'][1], 'ok')
        self.assertIn('모터 발열 약 3 W(추정)', short['정지 출력'][0])
        # 옛 제어기(발열·오래 유지 키 없음): 유지 시간만으로 같은 기준을 적용하고 발열은 꾸미지 않는다.
        old = rows(dict(STATUS, stop_output='hold', stop_output_a=30.0, stop_hold_basis='unsettled', stop_hold_s=11.0))
        self.assertEqual(old['정지 출력'][1], 'warn')
        self.assertNotIn('발열', old['정지 출력'][0])
        braking = rows(dict(STATUS, stop_output='brake', stop_output_a=20.0, stop_hold_s=0.0))
        self.assertEqual(braking['정지 출력'], ('brake 20.0 A · 근거 none', 'ok'))

    def test_keyboard_link_row(self):
        self.assertEqual(rows(STATUS)['제어 주기·LPP'], ('— (2026-11-17 이후 제어기)', 'unknown'))
        idle = rows(dict(STATUS, control_tick_max_ms=21.5, link_commands=0, link_cmd_max_gap_ms=0.0, link_expiries=0))
        self.assertEqual(idle['제어 주기·LPP'], ('제어 주기 최대 22 ms · LPP 명령 없음 · 만료 0 (최근 1초)', 'ok'))
        late = rows(dict(STATUS, control_tick_max_ms=55.0, link_commands=0, link_expiries=0))
        self.assertEqual(late['제어 주기·LPP'][1], 'warn')
        gap = rows(dict(STATUS, control_tick_max_ms=21.0, link_commands=15, link_cmd_max_gap_ms=350.0,
                        link_cmd_age_min_ms=3.0, link_cmd_age_max_ms=40.0, link_expiries=0))
        self.assertEqual(gap['제어 주기·LPP'][1], 'warn')
        self.assertIn('LPP 15개/초 · 최대 공백 350 ms · 지연 3~40 ms', gap['제어 주기·LPP'][0])

    def test_link_trace_parses_and_rates(self):
        link = vesc_status.link_trace_from_line(controller_line(200.25, LINK_TRACE))
        self.assertEqual((link['n'], link['gap_ms'], link['expired'], link['tick_max_ms'], link['stamp']),
                         (20.0, 62.0, 0.0, 22.0, 200.25))
        none_age = vesc_status.link_trace_from_line(controller_line(201., 'link_trace n=0 gap_ms=0 age_min_ms=na '
                                                                         'age_max_ms=na expired=0 tick_max_ms=20'))
        self.assertIsNone(none_age['age_min_ms'])
        self.assertIsNone(vesc_status.link_trace_from_line(controller_line(1., VESC_TRACE)))
        self.assertIsNone(vesc_status.link_trace_from_line('[planner] link_trace n=1'))
        title, text, level = vesc_status.link_row(link, 2.0)
        self.assertEqual((title, level), ('제어 주기·LPP', 'ok'))
        self.assertIn('평소(10초마다 기록) · 2초 전', text)
        bad = vesc_status.link_row(vesc_status.link_trace_from_line(controller_line(202., LINK_BAD)), 4.0)
        self.assertEqual(bad[2], 'warn')
        self.assertIn('만료 1', bad[1])
        self.assertIn('문제 있던 1초', bad[1])
        self.assertEqual(vesc_status.link_row(link, 30.0)[2], 'unknown')  # 평소 줄도 10초마다 오므로 오래됨
        self.assertEqual(vesc_status.link_row(None)[2], 'unknown')
        quiet = vesc_status.link_row(vesc_status.link_trace_from_line(controller_line(203., 'link_trace n=0 gap_ms=95000 '
                                                                       'age_min_ms=na age_max_ms=na expired=0 tick_max_ms=21')), 1.0)
        self.assertEqual(quiet[2], 'ok')  # 플래너가 꺼져 있는 것은 통신 문제로 보지 않는다
        self.assertIn('마지막 명령 95초 전', quiet[1])

    def test_autonomy_hold_row_from_traces(self):
        control = {'stop': 'hold', 'stop_a': 30.0}
        long_hold = dict(vesc_status.vesc_trace_from_line(controller_line(300., VESC_TRACE + ' hold_s=12.3')),
                         hold_basis='unsettled')
        row = dict((t, (x, l)) for t, x, l in vesc_status.trace_rows(control, long_hold))['정지 출력']
        self.assertEqual(row[1], 'warn')
        self.assertIn('12초째 · 모터 발열 약 15 W(추정) · 오래 유지 중', row[0])
        old = dict((t, (x, l)) for t, x, l in vesc_status.trace_rows(control,
                   vesc_status.vesc_trace_from_line(controller_line(301., VESC_TRACE))))['정지 출력']
        self.assertEqual(old[1], 'ok')  # 11-16 제어기: 유지 시간이 없으면 경고하지 않는다
        self.assertNotIn('초째', old[0])


SLIP_START = '[SLIP] stuck start: wheels 812 ERPM, iq 27.3 A, body 0.01 m/s (display only; output unchanged)'
SLIP_END = '[SLIP] backslide end after 1.2 s, 0.53 m backward'


class SlipRowsTest(unittest.TestCase):
    """2026-11-17 오후: 미끄러짐·막힘 표시 (2차 묶음 제어기 이후, 표시 전용)."""

    def test_slip_lines_parse_and_rate(self):
        start = vesc_status.slip_event_from_line(controller_line(500., SLIP_START).replace('[INFO]', '[WARN]'))
        self.assertEqual((start['kind'], start['phase'], start['stamp']), ('stuck', 'start', 500.0))
        title, text, level = vesc_status.slip_row(start, 2.0)
        self.assertEqual((title, level), ('미끄러짐·막힘', 'warn'))
        self.assertIn('헛돎·막힘 중 · wheels 812 ERPM, iq 27.3 A, body 0.01 m/s · 2초 전 (표시만, 출력 그대로)', text)
        self.assertEqual(vesc_status.slip_row(start, 45.0)[2], 'unknown')  # 끝 줄을 못 받은 오래된 시작
        end = vesc_status.slip_event_from_line(controller_line(501., SLIP_END))
        self.assertEqual((end['kind'], end['phase'], end['duration_s'], end['distance_m']), ('backslide', 'end', 1.2, 0.53))
        self.assertEqual(vesc_status.slip_row(end, 5.0), ('미끄러짐·막힘', '마지막: 뒤로 밀림 1.2초, 뒤로 0.53 m · 5초 전', 'ok'))
        self.assertEqual(vesc_status.slip_row(end, 90.0), ('미끄러짐·막힘', '최근 1분 사건 없음', 'ok'))
        self.assertEqual(vesc_status.slip_row(None)[2], 'unknown')
        self.assertIsNone(vesc_status.slip_event_from_line(controller_line(1., LINK_TRACE)))
        self.assertIsNone(vesc_status.slip_event_from_line('[planner] [SLIP] stuck start: x'))

    def test_keyboard_slip_row(self):
        self.assertEqual(rows(STATUS)['미끄러짐·막힘'][1], 'unknown')
        off = rows(dict(STATUS, slip_state='off', slip_off_reason='no localization pose'))
        self.assertEqual(off['미끄러짐·막힘'], ('판단 꺼짐 · no localization pose', 'unknown'))
        stuck = rows(dict(STATUS, slip_state='stuck', slip_state_s=2.4, body_speed_mps=0.01, slip_events=1))
        self.assertEqual(stuck['미끄러짐·막힘'][1], 'warn')
        self.assertIn('헛돎·막힘 중 2.4초', stuck['미끄러짐·막힘'][0])
        ok = rows(dict(STATUS, slip_state='ok', body_speed_mps=0.27, slip_events=0))
        self.assertEqual(ok['미끄러짐·막힘'], ('이상 없음 · 차체 0.27 m/s · 사건 0회', 'ok'))


class VescStatusWindowsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.windows = []

    def tearDown(self):
        for window in self.windows:
            window.close()
        self.app.processEvents()

    def test_keyboard_dialog_shows_received_status_only(self):
        dialog = DriveRecordDialog(None, None, None, None)
        self.windows.append(dialog)
        dialog.timer.stop()
        dialog.record_timer.stop()
        self.assertEqual(dialog.vesc_hint.text(), 'Pi4 연결 전')
        self.assertEqual(dialog.vesc_values['배터리'].text(), '—')
        snapshot = dict(status=STATUS, keys=dict(forward=False, left=False, right=False, back=False),
                        active=False, fresh=True, message='')
        dialog.client = type('Client', (), {'snapshot': lambda self: snapshot})()
        dialog.refresh_drive()
        self.assertIn('16.42 V', dialog.vesc_values['배터리'].text())
        self.assertIn('명령 0.512', dialog.vesc_values['서보'].text())
        self.assertIn(vesc_status.LEVEL_COLORS_LIGHT['ok'], dialog.vesc_values['배터리'].styleSheet())  # 밝은 테마
        self.assertFalse(dialog.vesc_diag_body.isVisibleTo(dialog))
        dialog.vesc_diag.setChecked(True)
        dialog.refresh_drive()
        self.assertTrue(dialog.vesc_diag_body.isVisibleTo(dialog))
        self.assertIn('30.1 / 31.2 / 30.8 ℃', dialog.vesc_diag_values['MOSFET 온도 1/2/3'].text())
        snapshot.update(fresh=False)
        dialog.refresh_drive()
        self.assertIn('상태 끊김', dialog.vesc_hint.text())
        self.assertIn(vesc_status.LEVEL_COLORS_LIGHT['unknown'], dialog.vesc_values['배터리'].styleSheet())
        snapshot.update(status=OLD_STATUS, fresh=True)
        dialog.refresh_drive()
        self.assertIn('제어기 배포 필요', dialog.vesc_hint.text())
        dialog.client = None

    def test_control_window_shows_vesc_trace_and_servo_command(self):
        child = ControlWindow()
        self.windows.append(child)
        now = time.time()
        text = controller_line(now, CONTROL_TRACE) + '\n' + controller_line(now + .001, VESC_TRACE)
        child.append_module_log(dict(kind='log', board='pi4', source='planning', t=now, seq=1, session='fixture', text=text))
        child.update_snapshot({})
        self.assertIn('16.42 V', child.vesc_values['배터리'].text())
        self.assertIn('31.2 ℃ · ok', child.vesc_values['ESC 온도'].text())
        self.assertIn('none 0.0 A · 근거 load', child.vesc_values['정지 출력'].text())
        self.assertEqual(child.servo.command, 0.512)
        self.assertIn('실측 조향각 아님', child.servo_status.text())
        # A stale vesc_trace never drives the servo bar.
        old = ControlWindow()
        self.windows.append(old)
        old.append_module_log(dict(kind='log', board='pi4', source='planning', t=now - 45, seq=1, session='old',
                                   text=controller_line(now - 45, VESC_TRACE)))
        old.update_snapshot({})
        self.assertIsNone(old.servo.command)
        self.assertIn('오래되어', old.servo_status.text())
        self.assertEqual(old.vesc_values['배터리'].text(), '— V · 입력 — A(VESC 추정) · — W')

    def test_control_window_shows_link_trace(self):
        child = ControlWindow()
        self.windows.append(child)
        now = time.time()
        child.update_snapshot({})
        self.assertIn('link_trace 없음', child.vesc_values['제어 주기·LPP'].text())
        self.assertIn('사건 없음', child.vesc_values['미끄러짐·막힘'].text())
        text = '\n'.join((controller_line(now, CONTROL_TRACE), controller_line(now + .001, VESC_TRACE),
                          controller_line(now + .002, LINK_BAD)))
        child.append_module_log(dict(kind='log', board='pi4', source='planning', t=now, seq=1, session='fixture', text=text))
        child.update_snapshot({})
        self.assertIn('만료 1', child.vesc_values['제어 주기·LPP'].text())
        self.assertIn('#fbbf24', child.vesc_values['제어 주기·LPP'].styleSheet())
        slip = controller_line(now + .003, SLIP_START).replace('[INFO]', '[WARN]')
        child.append_module_log(dict(kind='log', board='pi4', source='planning', t=now + .5, seq=2, session='fixture', text=slip))
        child.update_snapshot({})
        self.assertIn('헛돎·막힘 중', child.vesc_values['미끄러짐·막힘'].text())
        # A new controller start line drops the previous run's link figures.
        child.append_module_log(dict(kind='log', board='pi4', source='planning', t=now + 1, seq=3, session='fixture',
                                     text=controller_line(now + 1, 'starting mode=commission')))
        child.update_snapshot({})
        self.assertIn('link_trace 없음', child.vesc_values['제어 주기·LPP'].text())
        self.assertIn('사건 없음', child.vesc_values['미끄러짐·막힘'].text())


if __name__ == '__main__':
    unittest.main()
