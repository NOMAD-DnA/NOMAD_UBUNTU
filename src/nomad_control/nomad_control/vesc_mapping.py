"""Calibrated equivalent-steering / speed to VESC command conversion.

The VESC driver accepts a normalized servo position, not PWM microseconds.
All gains in this module must come from vehicle measurements.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VescCalibration:
    erpm_per_mps: float
    erpm_offset: float
    servo_per_rad: float
    servo_center: float
    duty_per_mps: float
    duty_kp: float
    duty_ki: float
    max_duty: float
    servo_min: float
    servo_max: float
    forward_sign: int = 1

    def validate(self, require_duty=True):
        numbers = (self.erpm_per_mps, self.erpm_offset, self.servo_per_rad, self.servo_center,
                   self.duty_per_mps, self.duty_kp, self.duty_ki,
                   self.max_duty, self.servo_min, self.servo_max)
        if not all(math.isfinite(v) for v in numbers):
            raise ValueError('VESC calibration must be finite')
        if (self.erpm_per_mps <= 0 or self.erpm_offset < 0 or self.servo_per_rad == 0
                or self.forward_sign not in (-1, 1)
                or self.duty_per_mps < 0 or self.duty_kp < 0 or self.duty_ki < 0
                or (require_duty and self.duty_per_mps + self.duty_kp + self.duty_ki <= 0)
                or not 0 < self.max_duty <= 1
                or not 0 <= self.servo_min < self.servo_center < self.servo_max <= 1):
            raise ValueError('VESC calibration has invalid limits or gains')

    def speed_from_erpm(self, erpm):
        if not math.isfinite(erpm):
            raise ValueError('non-finite VESC speed')
        normalized = self.forward_sign * erpm
        if abs(normalized) <= abs(self.erpm_offset):
            return 0.0
        return (normalized - math.copysign(self.erpm_offset, normalized)) / self.erpm_per_mps

    def erpm_from_speed(self, speed):
        if not math.isfinite(speed) or speed < 0:
            raise ValueError('invalid forward speed for VESC ERPM command')
        if speed == 0:
            return 0.0
        return self.forward_sign * (self.erpm_per_mps * speed + self.erpm_offset)

    def servo_from_angle(self, angle):
        if not math.isfinite(angle):
            raise ValueError('non-finite steering angle')
        position = self.servo_center + self.servo_per_rad * angle
        if not self.servo_min <= position <= self.servo_max:
            raise ValueError('steering angle exceeds calibrated servo range')
        return position


class DutyRegulator:
    def __init__(self, calibration):
        calibration.validate()
        self.cal = calibration
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, target_mps, measured_mps, dt):
        if not all(math.isfinite(v) for v in (target_mps, measured_mps, dt)) or dt <= 0:
            raise ValueError('invalid duty regulator input')
        if target_mps < 0:
            raise ValueError('reverse is disabled for the first real vehicle profile')
        if target_mps == 0:
            self.reset()
            return 0.0
        error = target_mps - measured_mps
        proposed_integral = self.integral + error * dt
        raw = (self.cal.duty_per_mps * target_mps + self.cal.duty_kp * error
               + self.cal.duty_ki * proposed_integral)
        # Do not accumulate integral in the direction of saturation.
        if (raw > self.cal.max_duty and error > 0) or (raw < 0 and error < 0):
            raw -= self.cal.duty_ki * (proposed_integral - self.integral)
        else:
            self.integral = proposed_integral
        return self.cal.forward_sign * min(self.cal.max_duty, max(0.0, raw))


def current_limit_exceeded(motor_a, input_a, limit_a):
    """Latch a stop when either reported current exceeds the configured ceiling."""
    if not all(math.isfinite(v) for v in (motor_a, input_a, limit_a)) or limit_a <= 0:
        return True
    return max(abs(motor_a), abs(input_a)) > limit_a
