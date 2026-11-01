"""Small dependency-free MDROBOT serial protocol implementation."""

import os
import select
import struct
import termios
import time
import fcntl
from dataclasses import dataclass


PC_ID = 172
CONTROLLER_ID = 183
MOTOR_ID = 1
PID_PARAMETER_REQUEST = 4
PID_TORQUE_OFF = 5
PID_SERIAL_CONTROL = 78
PID_START_STOP = 100
PID_CHANNEL_VELOCITY = {1: 130, 2: 131}
PID_COM_TARGET_SPEED = 180
PID_DUAL_VELOCITY = 207
PID_MAIN_DATA = 210


def checksum(data):
    return (-sum(data)) & 0xff


def build_packet(pid, payload=b''):
    packet = bytearray([CONTROLLER_ID, PC_ID, MOTOR_ID, pid, len(payload)])
    packet.extend(payload)
    packet.append(checksum(packet))
    return bytes(packet)


def build_parameter_request(pid):
    return build_packet(PID_PARAMETER_REQUEST, bytes([pid]))


def build_channel_velocity_command(channel, rpm):
    if channel not in PID_CHANNEL_VELOCITY:
        raise ValueError('channel must be 1 or 2')
    if not -32768 <= rpm <= 32767:
        raise ValueError('RPM command does not fit int16')
    return build_packet(PID_CHANNEL_VELOCITY[channel], struct.pack('<h', rpm))


def build_serial_arm_commands(start_mode):
    if start_mode not in (1, 2):
        raise ValueError('start_mode must be 1 (CCW) or 2 (CW)')
    return (
        build_packet(PID_SERIAL_CONTROL, b'\x01'),
        build_packet(PID_COM_TARGET_SPEED, struct.pack('<h', 0)),
        build_packet(PID_START_STOP, bytes([start_mode])),
    )


def build_dual_velocity_command(
        mot1_rpm, mot2_rpm, request_feedback=False,
        mot1_enable=True, mot2_enable=True):
    for rpm in (mot1_rpm, mot2_rpm):
        if not -32768 <= rpm <= 32767:
            raise ValueError('RPM command does not fit int16')
    payload = struct.pack(
        '<BhBhB', int(bool(mot1_enable)), mot1_rpm,
        int(bool(mot2_enable)), mot2_rpm,
        2 if request_feedback else 0)
    return build_packet(PID_DUAL_VELOCITY, payload)


def build_stop_packets():
    return (
        build_dual_velocity_command(0, 0),
        build_packet(PID_START_STOP, b'\x00'),
        build_packet(PID_TORQUE_OFF, b'\x00'),
    )


@dataclass(frozen=True)
class MotorFeedback:
    rpm: int
    current_a: float
    state: int
    position: int


def parse_main_data(payload):
    if len(payload) != 18:
        raise ValueError(f'PID210 payload length is {len(payload)}, not 18')

    def parse_at(offset):
        rpm, current_deciamp, state, position = struct.unpack_from(
            '<hHBi', payload, offset)
        return MotorFeedback(rpm, current_deciamp / 10.0, state, position)

    return {1: parse_at(0), 2: parse_at(9)}


class FrameParser:
    def __init__(self):
        self.header = bytes([PC_ID, CONTROLLER_ID])
        self.buffer = bytearray()

    def pop(self, expected_pid=None):
        while len(self.buffer) >= 6:
            start = self.buffer.find(self.header)
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                return None
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return None
            size = 6 + self.buffer[4]
            if len(self.buffer) < size:
                return None
            raw = bytes(self.buffer[:size])
            del self.buffer[:size]
            if sum(raw) & 0xff:
                continue
            if expected_pid is not None and raw[3] != expected_pid:
                continue
            return raw[3], raw[5:-1]
        return None


class Md200tSerial:
    """Open, command, and read one MD200T over an RS485 serial adapter."""

    def __init__(self, port, baudrate=57600):
        self.port = port
        self.baudrate = baudrate
        self.fd = -1
        self.parser = FrameParser()

    @property
    def is_open(self):
        return self.fd >= 0

    def open(self):
        if self.is_open:
            return
        self.fd = os.open(
            self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError(
                    f'serial port is already in use: {self.port}') from error
            attrs = termios.tcgetattr(self.fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0
            baud = {
                19200: termios.B19200,
                57600: termios.B57600,
                115200: termios.B115200,
            }.get(self.baudrate)
            if baud is None:
                raise ValueError(f'unsupported baudrate {self.baudrate}')
            attrs[4] = baud
            attrs[5] = baud
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except BaseException:
            os.close(self.fd)
            self.fd = -1
            raise

    def send(self, packet):
        if not self.is_open:
            raise RuntimeError('serial port is closed')
        offset = 0
        while offset < len(packet):
            offset += os.write(self.fd, packet[offset:])

    def read_frame(self, expected_pid, timeout=0.20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.parser.pop(expected_pid)
            if frame is not None:
                return frame[1]
            readable, _, _ = select.select(
                [self.fd], [], [], min(0.02, deadline - time.monotonic()))
            if readable:
                data = os.read(self.fd, 512)
                if data:
                    self.parser.buffer.extend(data)
        frame = self.parser.pop(expected_pid)
        return None if frame is None else frame[1]

    def read_feedback(self, retries=3, timeout=0.20):
        if retries < 1:
            raise ValueError('feedback retries must be at least one')
        if timeout <= 0.0:
            raise ValueError('feedback timeout must be positive')
        for _ in range(retries):
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.parser.buffer.clear()
            self.send(build_parameter_request(PID_MAIN_DATA))
            payload = self.read_frame(PID_MAIN_DATA, timeout=timeout)
            if payload is not None:
                return parse_main_data(payload)
        raise RuntimeError('no PID210 feedback from MD200T')

    def command(
            self, mot1_rpm, mot2_rpm, request_feedback=False,
            mot1_enable=True, mot2_enable=True):
        if request_feedback:
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.parser.buffer.clear()
        self.send(build_dual_velocity_command(
            mot1_rpm, mot2_rpm, request_feedback=request_feedback,
            mot1_enable=mot1_enable, mot2_enable=mot2_enable))
        if not request_feedback:
            return None
        payload = self.read_frame(PID_MAIN_DATA)
        if payload is None:
            raise RuntimeError('no PID210 feedback after PID207 command')
        return parse_main_data(payload)

    def arm_serial(self, start_mode):
        for packet in build_serial_arm_commands(start_mode):
            self.send(packet)

    def command_channel(self, channel, rpm):
        self.send(build_channel_velocity_command(channel, rpm))

    def stop(self):
        if self.is_open:
            for packet in build_stop_packets():
                self.send(packet)

    def close(self):
        if not self.is_open:
            return
        try:
            self.stop()
        finally:
            os.close(self.fd)
            self.fd = -1
