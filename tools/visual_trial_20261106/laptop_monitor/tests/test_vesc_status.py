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
        self.assertIn('#e2e8f0', dialog.vesc_values['배터리'].styleSheet())
        self.assertFalse(dialog.vesc_diag_body.isVisibleTo(dialog))
        dialog.vesc_diag.setChecked(True)
        dialog.refresh_drive()
        self.assertTrue(dialog.vesc_diag_body.isVisibleTo(dialog))
        self.assertIn('30.1 / 31.2 / 30.8 ℃', dialog.vesc_diag_values['MOSFET 온도 1/2/3'].text())
        snapshot.update(fresh=False)
        dialog.refresh_drive()
        self.assertIn('상태 끊김', dialog.vesc_hint.text())
        self.assertIn('#94a3b8', dialog.vesc_values['배터리'].styleSheet())
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


if __name__ == '__main__':
    unittest.main()
