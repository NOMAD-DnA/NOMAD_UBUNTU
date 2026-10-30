"""Pi 4 real-vehicle Control backend; run instead of the Gazebo controller.

Actuation requires explicit calibration and an independently verified VESC
command watchdog. No motor or servo command is sent while disarmed.
"""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from nomad_interfaces.msg import DriveCommand, ModuleStatus, VehicleState

from .controller import command_error, seconds
from .vesc_mapping import DutyRegulator, VescCalibration, current_limit_exceeded


class VescController(Node):
    def __init__(self):
        super().__init__('nomad_vesc_control')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.actuation_enabled = p('actuation_enabled', False)
        self.motor_mode = p('motor_mode', 'duty')
        self.calibration_valid = p('calibration_valid', False)
        self.watchdog_verified = p('watchdog_verified', False)
        self.base_frame = p('base_frame', 'base_link')
        self.current_limit_a = p('current_limit_a', 30.0)
        self.current_limit_verified = p('current_limit_verified', False)
        self.trial_candidate_conversion = p('trial_candidate_conversion', False)
        self.max_steer = p('max_steer', 0.0)
        self.command_timeout = p('command_wall_timeout', 0.35)
        self.telemetry_timeout = p('telemetry_wall_timeout', 0.5)
        self.rate = p('control_rate', 20.0)
        self.cal = VescCalibration(
            erpm_per_mps=p('erpm_per_mps', 0.0),
            erpm_offset=p('erpm_offset', 0.0),
            servo_per_rad=p('servo_per_rad', 0.0),
            servo_center=p('servo_center', 0.5),
            duty_per_mps=p('duty_per_mps', 0.0),
            duty_kp=p('duty_kp', 0.0),
            duty_ki=p('duty_ki', 0.0),
            max_duty=p('max_duty', 0.0),
            servo_min=p('servo_min', 0.0),
            servo_max=p('servo_max', 1.0),
            forward_sign=p('forward_sign', 1))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.command_timeout, self.telemetry_timeout, self.rate)):
            raise ValueError('VESC timeout/rate must be finite and positive')
        if self.motor_mode not in ('duty', 'erpm'):
            raise ValueError('motor_mode must be duty or erpm')
        if self.calibration_valid or self.trial_candidate_conversion:
            self.cal.validate(require_duty=self.motor_mode == 'duty')
        if not math.isfinite(self.current_limit_a) or not 0 < self.current_limit_a <= 30.0:
            raise ValueError('current_limit_a must be >0 and <=30 A')
        if self.actuation_enabled and (not (self.calibration_valid or self.trial_candidate_conversion)
                                       or not self.watchdog_verified or not self.current_limit_verified
                                       or not math.isfinite(self.max_steer) or self.max_steer <= 0):
            raise ValueError('VESC actuation requires conversion, steering range, verified 30A firmware limit and watchdog')
        self.regulator = (DutyRegulator(self.cal) if (self.calibration_valid or self.trial_candidate_conversion)
                          and self.motor_mode == 'duty' else None)
        self.command = None
        self.command_wall = -math.inf
        self.latest_command_stamp = -math.inf
        self.telemetry_wall = -math.inf
        self.speed = 0.0
        self.fault_code = 0
        self.current_tripped = False
        self.motor_current = 0.0
        self.input_current = 0.0
        self.last_tick = time.monotonic()
        self.duty_pub = self.create_publisher(Float64, 'commands/motor/duty_cycle', 10)
        self.erpm_pub = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.current_pub = self.create_publisher(Float64, 'commands/motor/current', 10)
        self.servo_pub = self.create_publisher(Float64, 'commands/servo/position', 10)
        self.state_pub = self.create_publisher(VehicleState, '/nomad/control/vehicle_state', 10)
        self.status_pub = self.create_publisher(ModuleStatus, '/nomad/control/status', 10)
        self.create_subscription(DriveCommand, '/nomad/planning/drive_command', self.on_command, 1)
        self.create_subscription(VescStateStamped, 'sensors/core', self.on_telemetry,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self.rate, self.tick, clock=self.wall_clock)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stop(self):
        if self.regulator is not None:
            self.regulator.reset()
        if self.actuation_enabled:
            if self.motor_mode == 'duty':
                self.duty_pub.publish(Float64(data=0.0))
            else:
                self.erpm_pub.publish(Float64(data=0.0))
            self.current_pub.publish(Float64(data=0.0))
            self.servo_pub.publish(Float64(data=self.cal.servo_center))

    def on_command(self, msg):
        stamp = seconds(msg.header.stamp)
        if stamp < self.latest_command_stamp:
            return
        if stamp == self.latest_command_stamp:
            if msg.stop_requested:
                self.command = None
                self.stop()
            return
        error = command_error(msg, self.now(), math.inf, self.max_steer,
                              self.base_frame)
        if error or msg.target_speed < 0:
            self.command = None
            self.stop()
            return
        self.latest_command_stamp = stamp
        self.command, self.command_wall = msg, time.monotonic()
        if msg.stop_requested:
            self.stop()

    def on_telemetry(self, msg):
        if not (self.calibration_valid or self.trial_candidate_conversion):
            return
        stamp = seconds(msg.header.stamp)
        if not -.05 <= self.now() - stamp <= self.telemetry_timeout:
            return
        try:
            speed = self.cal.speed_from_erpm(msg.state.speed)
        except ValueError:
            return
        motor_current = msg.state.current_motor
        input_current = msg.state.current_input
        if not all(math.isfinite(v) for v in (motor_current, input_current)):
            self.current_tripped = True
            self.stop()
            return
        self.motor_current = motor_current
        self.input_current = input_current
        if current_limit_exceeded(motor_current, input_current, self.current_limit_a):
            self.current_tripped = True
            self.stop()
        self.speed = speed
        self.fault_code = msg.state.fault_code
        self.telemetry_wall = time.monotonic()

    def tick(self):
        wall = time.monotonic()
        dt = min(0.2, max(1e-3, wall - self.last_tick))
        self.last_tick = wall
        fresh = wall - self.telemetry_wall <= self.telemetry_timeout
        armed = bool(self.actuation_enabled and (self.calibration_valid or self.trial_candidate_conversion)
                     and self.watchdog_verified and self.current_limit_verified)
        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.base_frame
        state.speed = self.speed
        state.speed_valid = fresh
        # VESC telemetry does not measure the steering joint. Never echo a
        # servo target into the measured VehicleState field.
        state.steering_valid = False
        state.steering_angle = 0.0
        state.fault = bool(self.current_tripped or (fresh and self.fault_code != 0))
        state.ready = bool(armed and fresh and not state.fault)
        state.detail = ('VESC speed measured; steering joint unmeasured; candidate conversion'
                        if state.ready and self.trial_candidate_conversion else
                        'VESC speed measured; steering joint unmeasured' if state.ready else
                        'STOP: current exceeded configured limit' if self.current_tripped else
                        'WAIT: conversion/current-limit/watchdog/telemetry/fault')
        self.state_pub.publish(state)
        reason = None
        if not state.ready:
            reason = state.detail
        elif self.command is None:
            reason = 'WAIT: DriveCommand'
        elif wall - self.command_wall > self.command_timeout:
            reason = 'STOP: command wall timeout'
        else:
            reason = command_error(self.command, self.now(), math.inf,
                                   self.max_steer, self.base_frame)
        if reason is None and self.command.stop_requested:
            reason = 'STOP: planning request'
        if reason is None and self.speed < -0.03:
            reason = 'STOP: reverse motion detected'
        if reason is None:
            try:
                servo = self.cal.servo_from_angle(self.command.target_steering_angle)
                motor = (self.regulator.step(self.command.target_speed, self.speed, dt)
                         if self.motor_mode == 'duty' else
                         self.cal.erpm_from_speed(self.command.target_speed))
            except ValueError as exc:
                reason = f'STOP: {exc}'
        if reason is None:
            self.servo_pub.publish(Float64(data=servo))
            if self.motor_mode == 'duty':
                self.duty_pub.publish(Float64(data=motor))
            else:
                self.erpm_pub.publish(Float64(data=motor))
            reason = 'RUNNING'
        else:
            self.stop()
        status = ModuleStatus()
        status.header = state.header
        status.state = (ModuleStatus.FAULT if state.fault else
                        ModuleStatus.READY if state.ready else ModuleStatus.WAITING)
        status.valid_for.nanosec = 600_000_000
        status.detail = reason
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = VescController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
