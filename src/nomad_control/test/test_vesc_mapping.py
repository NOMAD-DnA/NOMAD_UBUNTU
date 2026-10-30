import pytest

from nomad_control.vesc_mapping import DutyRegulator, VescCalibration, current_limit_exceeded


def calibration(**changes):
    values = dict(erpm_per_mps=4614.0, erpm_offset=0.0,
                  servo_per_rad=-1.2135, servo_center=0.5304,
                  duty_per_mps=0.2, duty_kp=0.1, duty_ki=0.05,
                  max_duty=0.3, servo_min=0.0, servo_max=1.0,
                  forward_sign=-1)
    values.update(changes)
    return VescCalibration(**values)


def test_existing_example_ratios_and_forward_sign():
    cal = calibration()
    cal.validate()
    assert cal.speed_from_erpm(-4614) == pytest.approx(1.0)
    assert cal.erpm_from_speed(0.1) == pytest.approx(-461.4)
    assert cal.erpm_from_speed(0.0) == 0.0
    assert cal.servo_from_angle(0.1) == pytest.approx(0.40905)
    assert cal.servo_from_angle(0) == pytest.approx(0.5304)


def test_observed_pi4_forward_sign():
    cal = calibration(forward_sign=1)
    assert cal.speed_from_erpm(-20000) < 0
    assert cal.erpm_from_speed(0.1) == pytest.approx(461.4)
    assert DutyRegulator(cal).step(0.1, 0.0, 0.05) > 0


def test_duty_feedback_saturation_stop_and_reverse_rejection():
    regulator = DutyRegulator(calibration())
    assert -0.3 <= regulator.step(0.5, 0.0, 0.05) < 0
    assert regulator.step(5.0, 0.0, 0.05) == -0.3
    assert regulator.step(0.0, 0.0, 0.05) == 0.0
    assert regulator.integral == 0.0
    with pytest.raises(ValueError):
        regulator.step(-0.1, 0.0, 0.05)


def test_bad_calibration_or_out_of_range_steering_fails():
    with pytest.raises(ValueError):
        calibration(erpm_per_mps=0).validate()
    with pytest.raises(ValueError):
        calibration(servo_per_rad=0).validate()
    with pytest.raises(ValueError):
        calibration().servo_from_angle(1.0)


def test_motor_and_input_current_trip_at_30a():
    assert not current_limit_exceeded(29.9, 12.0, 30.0)
    assert not current_limit_exceeded(30.0, 30.0, 30.0)
    assert current_limit_exceeded(30.1, 10.0, 30.0)
    assert current_limit_exceeded(10.0, -30.1, 30.0)
    assert current_limit_exceeded(float("nan"), 1.0, 30.0)
