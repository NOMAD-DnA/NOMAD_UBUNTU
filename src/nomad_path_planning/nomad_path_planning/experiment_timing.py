"""Opt-in experiment timing. Does not alter planner inputs, outputs, or limits."""
import csv
import os
import time
from contextlib import contextmanager
from pathlib import Path

_stream = None
_writer = None


@contextmanager
def measured(node, stage):
    global _stream, _writer
    directory = os.environ.get('NOMAD_EXPERIMENT_TIMING_DIR')
    if not directory:
        yield
        return
    if _writer is None:
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        _stream = (path / f'{node.get_name()}_{os.getpid()}.csv').open('a', buffering=1)
        _writer = csv.writer(_stream)
        _writer.writerow(['wall_epoch_ns', 'sim_ns', 'stage', 'elapsed_wall_ns', 'thread_cpu_ns', 'exception'])
    epoch = time.time_ns()
    sim = node.get_clock().now().nanoseconds
    start, cpu = time.perf_counter_ns(), time.thread_time_ns()
    error = ''
    try:
        yield
    except BaseException as exc:
        error = type(exc).__name__
        raise
    finally:
        wall_elapsed, cpu_elapsed = time.perf_counter_ns()-start, time.thread_time_ns()-cpu
        _writer.writerow([epoch, sim, stage, wall_elapsed, cpu_elapsed, error])
