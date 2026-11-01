import struct
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import Twist

from roka_md_driver.drive_math import diagonal_covariance, twist_to_wheel_rpm
from roka_md_driver.md_driver_node import ROKAMdDriver
from roka_md_driver.md_driver_node import MotorSafetyFault
from roka_md_driver.md_driver_node import is_transient_feedback_timeout
from roka_md_driver.md_driver_node import hold_feedback_stopped
from roka_md_driver.protocol import Md200tSerial, MotorFeedback
from roka_md_driver.protocol import build_dual_velocity_command, parse_main_data
from roka_md_driver.protocol import build_channel_velocity_command
from roka_md_driver.protocol import build_serial_arm_commands


def test_pid207_packet_matches_mdrobot_protocol():
    packet = build_dual_velocity_command(-50, -50, request_feedback=True)
    assert packet.hex() == 'b7ac01cf0701ceff01ceff0228'
    assert sum(packet) & 0xff == 0


def test_same_side_command_uses_same_raw_rpm_before_controller_pid16():
    packet = build_dual_velocity_command(14, 14, request_feedback=False)
    assert packet.hex() == 'b7ac01cf07010e00010e0000a8'


def test_individual_pid207_disables_other_channel():
    mot1 = build_dual_velocity_command(
        14, 0, mot1_enable=True, mot2_enable=False,
        request_feedback=True)
    mot2 = build_dual_velocity_command(
        0, 14, mot1_enable=False, mot2_enable=True,
        request_feedback=True)
    assert mot1.hex() == 'b7ac01cf07010e0000000002b5'
    assert mot2.hex() == 'b7ac01cf07000000010e0002b5'


def test_individual_channel_packets_and_serial_arm_sequence():
    assert build_channel_velocity_command(1, 28).hex() == 'b7ac0182021c00fc'
    assert build_channel_velocity_command(2, -28).hex() == 'b7ac018302e4ff34'
    assert [packet.hex() for packet in build_serial_arm_commands(1)] == [
        'b7ac014e01014c',
        'b7ac01b4020000e6',
        'b7ac0164010136',
    ]


def test_pid210_feedback_decode():
    payload = bytearray(18)
    struct.pack_into('<hHBi', payload, 0, 28, 7, 0, 12)
    struct.pack_into('<hHBi', payload, 9, -28, 8, 0, -12)
    feedback = parse_main_data(payload)
    assert feedback[1].rpm == 28
    assert feedback[1].current_a == pytest.approx(0.7)
    assert feedback[2].position == -12


def test_point_three_mps_is_capped_at_28_rpm():
    left, right = twist_to_wheel_rpm(0.3, 0.0, 0.60, 0.103, 28.0)
    expected = 0.3 * 60.0 / (2.0 * 3.141592653589793 * 0.103)
    assert left == pytest.approx(expected)
    assert right == pytest.approx(expected)


def test_four_wheel_config_inverts_only_angular_command():
    config_path = (
        Path(__file__).parents[1] / 'config' /
        'mdh250_md200t_four_wheel.yaml')
    config = yaml.safe_load(config_path.read_text())
    params = config['roka_md_driver']['ros__parameters']
    assert params['angular_command_sign'] == 1
    assert params['left_mot1_command_sign'] == 1
    assert params['left_mot2_command_sign'] == -1
    assert params['right_mot1_command_sign'] == 1
    assert params['right_mot2_command_sign'] == -1
    assert params['feedback_timeout_sec'] >= 3.0 / params['feedback_rate_hz']
    assert params['feedback_read_retries'] == 1
    assert params['feedback_read_timeout_sec'] < 1.0 / params['feedback_rate_hz']
    assert params['startup_retry_interval_sec'] > 0.0
    assert (
        params['startup_retry_log_interval_sec'] >=
        params['startup_retry_interval_sec']
    )
    assert params['feedback_overspeed_limit_rpm'] == pytest.approx(180.0)
    assert params['feedback_overspeed_limit_rpm'] > params['max_wheel_rpm']
    assert params['abort_current_a'] == pytest.approx(9.5)
    assert params['hold_maximum_rpm'] == pytest.approx(2.0)
    assert params['hold_stable_sec'] == pytest.approx(0.5)
    assert any(params['twist_covariance_diagonal'])


def test_feedback_guard_allows_transient_wheelspin_but_keeps_hard_limit():
    driver = ROKAMdDriver.__new__(ROKAMdDriver)
    driver.abort_current = 9.5
    driver.feedback_overspeed_limit = 90.0
    feedback = {
        1: MotorFeedback(rpm=89, current_a=9.5, state=0, position=0),
        2: MotorFeedback(rpm=-89, current_a=9.5, state=0, position=0),
    }
    driver._check_feedback('left', feedback)

    feedback[1] = MotorFeedback(rpm=91, current_a=1.0, state=0, position=0)
    with pytest.raises(MotorSafetyFault, match='90 RPM feedback limit'):
        driver._check_feedback('left', feedback)

    feedback[1] = MotorFeedback(rpm=56, current_a=9.6, state=0, position=0)
    with pytest.raises(MotorSafetyFault, match='current 9.6 A exceeds limit'):
        driver._check_feedback('left', feedback)


def test_hold_requires_all_four_motors_to_report_near_zero_rpm():
    stopped = {
        'left': {
            1: MotorFeedback(rpm=1, current_a=0.5, state=0, position=10),
            2: MotorFeedback(rpm=-2, current_a=0.5, state=0, position=-10),
        },
        'right': {
            1: MotorFeedback(rpm=0, current_a=0.5, state=0, position=20),
            2: MotorFeedback(rpm=2, current_a=0.5, state=0, position=-20),
        },
    }
    assert hold_feedback_stopped(stopped, 2.0)
    stopped['right'][2] = MotorFeedback(
        rpm=3, current_a=0.5, state=0, position=-20
    )
    assert not hold_feedback_stopped(stopped, 2.0)

def test_runtime_communication_loss_stops_and_retries_without_latched_fault(
        monkeypatch):
    driver = ROKAMdDriver.__new__(ROKAMdDriver)
    driver.fault = False
    driver.startup_retry_interval = 2.0
    driver.cmd = Twist()
    driver.cmd.linear.x = 0.3
    driver.last_cmd_time = 99.0
    driver.feedback_tick = 4
    driver.last_feedback_time = {'left': 98.0, 'right': 98.0}
    published = []
    warnings = []
    closed = []
    driver.connected_pub = SimpleNamespace(
        publish=lambda message: published.append(message.data)
    )
    monkeypatch.setattr(
        driver,
        'get_logger',
        lambda: SimpleNamespace(warning=warnings.append),
    )
    monkeypatch.setattr(driver, '_close_hardware', lambda: closed.append(True))
    monkeypatch.setattr(
        'roka_md_driver.md_driver_node.time.monotonic', lambda: 100.0
    )

    driver._begin_communication_recovery('serial timeout')

    assert not driver.fault
    assert driver.cmd.linear.x == 0.0
    assert driver.last_cmd_time is None
    assert driver.feedback_tick == 0
    assert driver.last_feedback_time == {}
    assert driver.next_startup_attempt == pytest.approx(102.0)
    assert published == [False]
    assert closed == [True]
    assert 'automatic reconnect' in warnings[0]


def test_first_feedback_failure_closes_the_just_opened_serial_port(monkeypatch):
    class LateController:
        def __init__(self):
            self.opened = False
            self.closed = False

        def open(self):
            self.opened = True

        def read_feedback(self):
            raise RuntimeError('no PID210 feedback from MD200T')

        def close(self):
            self.closed = True

    controller = LateController()
    driver = ROKAMdDriver.__new__(ROKAMdDriver)
    driver.controllers = {}
    driver.feedback = {}
    driver.last_feedback_time = {}
    monkeypatch.setattr(
        driver, 'get_parameter',
        lambda name: SimpleNamespace(value=(
            57600 if name == 'baudrate' else '/dev/fake-md200t')),
    )
    monkeypatch.setattr(driver, '_active_sides', lambda: ('left',))
    monkeypatch.setattr(
        'roka_md_driver.md_driver_node.Md200tSerial',
        lambda _port, _baud: controller,
    )

    with pytest.raises(RuntimeError, match='no PID210 feedback'):
        driver._open_hardware()

    assert controller.opened
    assert controller.closed
    assert driver.controllers == {}


def test_only_missing_pid210_reply_is_a_transient_feedback_timeout():
    assert is_transient_feedback_timeout(
        RuntimeError('no PID210 feedback from MD200T'))
    assert not is_transient_feedback_timeout(
        RuntimeError('left MOT1 state=1'))
    assert not is_transient_feedback_timeout(
        RuntimeError('left MOT1 current 9.6 A exceeds limit'))
    assert not is_transient_feedback_timeout(OSError('USB disconnected'))
    assert not is_transient_feedback_timeout(
        ValueError('PID210 payload length is 12, not 18'))


def test_hardware_feedback_freshness_allows_one_miss_then_expires():
    driver = ROKAMdDriver.__new__(ROKAMdDriver)
    driver.simulation = False
    driver.layout = 'dual'
    driver.single_side = 'right'
    driver.controllers = {'left': object(), 'right': object()}
    driver.feedback_timeout = 1.0
    driver.last_feedback_time = {'left': 10.0, 'right': 10.4}

    assert driver._hardware_feedback_fresh(10.9)
    assert not driver._hardware_feedback_fresh(11.01)


def test_runtime_feedback_read_uses_bounded_timeout(monkeypatch):
    controller = Md200tSerial('/dev/null')
    controller.fd = 123
    payload = bytearray(18)
    calls = []

    monkeypatch.setattr(
        'roka_md_driver.protocol.termios.tcflush',
        lambda _fd, _queue: None,
    )
    monkeypatch.setattr(controller, 'send', lambda packet: calls.append(packet))

    def read_frame(expected_pid, timeout):
        calls.append((expected_pid, timeout))
        return payload

    monkeypatch.setattr(controller, 'read_frame', read_frame)
    feedback = controller.read_feedback(retries=1, timeout=0.05)

    assert len(feedback) == 2
    assert calls[-1][1] == pytest.approx(0.05)


def test_feedback_read_rejects_invalid_retry_budget():
    controller = Md200tSerial('/dev/null')
    with pytest.raises(ValueError, match='at least one'):
        controller.read_feedback(retries=0)
    with pytest.raises(ValueError, match='must be positive'):
        controller.read_feedback(timeout=0.0)


def test_covariance_diagonal_is_expanded_without_claiming_unobserved_axes():
    covariance = diagonal_covariance([0.01, 1e6, 1e6, 1e6, 1e6, 0.04])
    assert len(covariance) == 36
    assert covariance[0] == pytest.approx(0.01)
    assert covariance[7] == pytest.approx(1e6)
    assert covariance[35] == pytest.approx(0.04)
    assert sum(value != 0.0 for value in covariance) == 6


def test_invalid_covariance_is_rejected():
    with pytest.raises(ValueError):
        diagonal_covariance([0.0] * 5)
    with pytest.raises(ValueError):
        diagonal_covariance([0.0, 0.0, -1.0, 0.0, 0.0, 0.0])
