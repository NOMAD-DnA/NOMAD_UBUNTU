#!/usr/bin/env python3
"""VESC 5.03 config backup / narrowly scoped timeout update. Never commands motion."""
import argparse
import binascii
import fcntl
import json
import os
from pathlib import Path
import select
import struct
import termios
import time
import tty

PORT = '/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00'

class Link:
    def __init__(self, port):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.ioctl(self.fd, termios.TIOCEXCL)
        tty.setraw(self.fd)
        attr = termios.tcgetattr(self.fd)
        attr[4] = attr[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.buffer = bytearray()
    def close(self):
        fcntl.ioctl(self.fd, termios.TIOCNXCL)
        os.close(self.fd)
    def request(self, payload):
        length = len(payload)
        header = bytes([2, length]) if length < 256 else bytes([3]) + struct.pack('>H', length)
        packet = header + payload + struct.pack('>H', binascii.crc_hqx(payload, 0)) + b'\x03'
        sent = 0
        while sent < len(packet):
            select.select([], [self.fd], [], 1)
            sent += os.write(self.fd, packet[sent:])
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            while len(self.buffer) >= 2:
                start = self.buffer[0]
                if start not in (2, 3): del self.buffer[0]; continue
                header_size = 2 if start == 2 else 3
                if len(self.buffer) < header_size: break
                size = int.from_bytes(self.buffer[1:header_size], 'big')
                if size > 4096: raise RuntimeError('Invalid frame length')
                total = header_size + size + 3
                if len(self.buffer) < total: break
                data = bytes(self.buffer[header_size:header_size + size])
                crc = int.from_bytes(self.buffer[header_size+size:header_size+size+2], 'big')
                end = self.buffer[total - 1]
                del self.buffer[:total]
                if end != 3 or crc != binascii.crc_hqx(data, 0): raise RuntimeError('Bad CRC/frame')
                if data and data[0] == payload[0]: return data
            if select.select([self.fd], [], [], 0.1)[0]: self.buffer.extend(os.read(self.fd, 8192))
        raise TimeoutError(f'No reply for command {payload[0]}')

def read_app(data):
    body = data[1:]
    if len(body) < 30 or struct.unpack('>I', body[:4])[0] != 3733512279:
        raise RuntimeError('Unsupported App configuration signature; no writes allowed')
    return dict(controller_id=body[4], timeout_ms=int.from_bytes(body[5:9], 'big'),
                timeout_brake_a=struct.unpack('>f', body[9:13])[0], servo_output=bool(body[27]),
                app_to_use=body[29])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--set-timeout-ms', type=int)
    parser.add_argument('--bench-current-a', type=float)
    parser.add_argument('--require-current-a', type=float)
    parser.add_argument('--require-timeout-ms', type=int)
    args = parser.parse_args()
    if args.set_timeout_ms is not None and not 100 <= args.set_timeout_ms <= 1000:
        parser.error('Timeout must be between 100 and 1000 ms')
    if args.bench_current_a is not None and not 0 < args.bench_current_a <= 50:
        parser.error('Requested current must be >0 and <=50A')
    lock = open(str(Path.home()/'vesc_ws'/'device.lock'), 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    link = Link(args.port)
    try:
        fw = link.request(b'\0')
        if len(fw) < 3 or fw[1:3] != b'\x05\x03':
            raise RuntimeError('Only observed firmware 5.03 is supported')
        directory = Path.home()/'vesc_ws'/'backups'/time.strftime('%Y%m%d_%H%M%S')
        directory.mkdir(parents=True, exist_ok=False)
        app = link.request(bytes([17]))
        motor = link.request(bytes([14]))
        (directory/'app_payload.bin').write_bytes(app)
        (directory/'motor_payload.bin').write_bytes(motor)
        (directory/'firmware_payload.bin').write_bytes(fw)
        result = read_app(app)
        m = motor[1:]
        if len(m) >= 28 and struct.unpack('>I', m[:4])[0] == 2525666056:
            result['motor'] = dict(motor_type=m[6], current_max_a=struct.unpack('>f',m[8:12])[0],
                current_min_a=struct.unpack('>f',m[12:16])[0], battery_max_a=struct.unpack('>f',m[16:20])[0],
                battery_regen_a=struct.unpack('>f',m[20:24])[0])
        if args.require_current_a is not None:
            if 'motor' not in result:
                raise RuntimeError('Motor configuration cannot be verified')
            limit = args.require_current_a
            values = result['motor']
            if not 0 < limit <= 30 or any(abs(values[key]) > limit + 1e-3 for key in
                    ('current_max_a', 'current_min_a', 'battery_max_a', 'battery_regen_a')):
                raise RuntimeError('VESC firmware current limit is not at or below requested ceiling')
        if args.require_timeout_ms is not None and not 100 <= result['timeout_ms'] <= args.require_timeout_ms:
            raise RuntimeError('VESC firmware timeout is not verified')
        if args.bench_current_a is not None:
            if 'motor' not in result: raise RuntimeError('Unknown Motor signature; no writes allowed')
            updated_motor = bytearray(m)
            updated_motor[8:24] = struct.pack('>ffff', args.bench_current_a, -args.bench_current_a,
                                            args.bench_current_a, 0.0)
            link.request(bytes([13]) + updated_motor)
            actual_motor = link.request(bytes([14]))
            (directory/'motor_after_payload.bin').write_bytes(actual_motor)
            if actual_motor[1:] != updated_motor: raise RuntimeError('Motor config readback mismatch')
            result['motor_after'] = dict(current_max_a=args.bench_current_a, current_min_a=-args.bench_current_a,
                                        battery_max_a=args.bench_current_a, battery_regen_a=0.0)
        if args.set_timeout_ms is not None:
            updated = bytearray(app[1:])
            updated[5:9] = struct.pack('>I', args.set_timeout_ms)
            updated[9:13] = struct.pack('>f', 0.0)
            # Preserve every unrelated byte; only timeout and its brake current change.
            link.request(bytes([16]) + updated)
            actual = link.request(bytes([17]))
            (directory/'app_after_payload.bin').write_bytes(actual)
            if actual[1:] != updated: raise RuntimeError('App readback mismatch; inspect backup before proceeding')
            result['after'] = read_app(actual)
        result['backup'] = str(directory)
        (directory/'summary.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally: link.close()

if __name__ == '__main__': main()
