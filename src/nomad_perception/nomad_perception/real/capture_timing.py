"""Capture provenance and same-host processing intervals, in integer nanoseconds.

ROS capture time and the local monotonic clock are never subtracted from one
another. Measurement age is meaningful only after the source clocks are verified.
"""
from copy import deepcopy
from numbers import Integral
import time


def positive_ns(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value <= 0:
        raise ValueError('Capture timestamp must be positive integer nanoseconds')
    return int(value)


def trace(capture_ns, *, received_ros_ns=None, received_monotonic_ns=None,
          point_stamps_ns=None, receipt_scope='sensor_callback'):
    value = dict(schema='capture-timing-v1', source_stamp_ns=positive_ns(capture_ns),
                 received_ros_ns=int(time.time_ns() if received_ros_ns is None else received_ros_ns),
                 received_monotonic_ns=int(time.monotonic_ns() if received_monotonic_ns is None else received_monotonic_ns),
                 receipt_scope=receipt_scope)
    return capture_range(value, point_stamps_ns)


def capture_range(value, point_stamps_ns=None):
    result = deepcopy(value)
    stamps = [] if point_stamps_ns is None else list(point_stamps_ns)
    result['capture_first_ns'] = min(map(positive_ns, stamps)) if stamps else positive_ns(result['source_stamp_ns'])
    result['capture_last_ns'] = max(map(positive_ns, stamps)) if stamps else positive_ns(result['source_stamp_ns'])
    return result


def stage(value, name, *, monotonic_ns=None, ros_ns=None):
    result = deepcopy(value)
    result[name + '_monotonic_ns'] = int(time.monotonic_ns() if monotonic_ns is None else monotonic_ns)
    if ros_ns is not None:
        result[name + '_ros_ns'] = int(ros_ns)
    return result


def summary(value, *, now_ros_ns=None, now_monotonic_ns=None):
    if value is None:
        return None
    result = deepcopy(value)
    now_ros_ns = time.time_ns() if now_ros_ns is None else int(now_ros_ns)
    now_monotonic_ns = time.monotonic_ns() if now_monotonic_ns is None else int(now_monotonic_ns)
    result['oldest_point_age_ms'] = (now_ros_ns - value['capture_first_ns']) / 1e6
    result['newest_point_age_ms'] = (now_ros_ns - value['capture_last_ns']) / 1e6
    result['since_receive_ms'] = (now_monotonic_ns - value['received_monotonic_ns']) / 1e6
    intervals = (
        ('semantic_wait_ms', 'received', 'semantic_ready'),
        ('worker_queue_ms', 'submitted', 'worker_started'),
        ('worker_processing_ms', 'worker_started', 'worker_completed'),
        ('local_pipeline_ms', 'received', 'worker_completed'),
    )
    for label, first, last in intervals:
        a, b = value.get(first + '_monotonic_ns'), value.get(last + '_monotonic_ns')
        if a is not None and b is not None:
            if b < a:
                raise ValueError('Processing stages use inconsistent monotonic clocks')
            result[label] = (b - a) / 1e6
    return result
