"""Guarded ROS 2 bridge for one or two MD200T controllers."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from roka_robot_description.hardware_config import load_hardware_config

from roka_md_driver.drive_math import (
    diagonal_covariance,
    rpm_to_mps,
    twist_to_wheel_rpm,
)
from roka_md_driver.protocol import Md200tSerial


PID210_FEEDBACK_TIMEOUT = 'no PID210 feedback from MD200T'


class MotorSafetyFault(RuntimeError):
    """Latched fault reported by the motor controller or feedback guards."""


def is_transient_feedback_timeout(error):
    """Return true only for an exhausted PID210 read with no reply.

    A single missing sample is not a controller fault.  The driver keeps the
    last valid sample until ``feedback_timeout_sec`` expires.  Packet errors,
    motor state/current/overspeed faults, and serial I/O errors remain hard
    faults and are never classified as transient here.
    """
    return (
        isinstance(error, RuntimeError)
        and str(error) == PID210_FEEDBACK_TIMEOUT
    )


def hold_feedback_stopped(feedback_by_side, maximum_rpm):
    """Return true only when every configured motor reports near-zero RPM."""
    if not feedback_by_side or maximum_rpm < 0.0:
        return False
    motors = [
        motor
        for side_feedback in feedback_by_side.values()
        for motor in side_feedback.values()
    ]
    return bool(motors) and all(abs(float(motor.rpm)) <= maximum_rpm for motor in motors)


class ROKAMdDriver(Node):
    def __init__(self):
        super().__init__('roka_md_driver')
        hardware = load_hardware_config()
        defaults = {
            'simulation_mode': True,
            'allow_motion': False,
            'hardware_layout': 'dual',
            'single_side': 'right',
            'left_serial_port': (
                '/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_'
                'A76BHBER-if00-port0'),
            'right_serial_port': (
                '/dev/serial/by-id/usb-FTDI_USB_to_RS-422_485_Adapter_'
                'A751CYHM-if00-port0'),
            'baudrate': 57600,
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/odom',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'wheel_radius': hardware.wheel_radius_m,
            # Effective skid-steer separation, not physical tire geometry.
            'wheel_separation': hardware.effective_drive_separation_m,
            'max_linear_speed': 0.30,
            'max_angular_speed': 0.60,
            'angular_command_sign': 1,
            'max_wheel_rpm': 28.0,
            'command_timeout_sec': 0.50,
            'command_rate_hz': 20.0,
            'feedback_rate_hz': 5.0,
            'feedback_timeout_sec': 1.0,
            # A normal FTDI/MD200T PID210 reply was measured at 7--24 ms.
            # Runtime reads get one bounded attempt so one delayed controller
            # cannot stall both sides and the 20 Hz command loop. Startup keeps
            # the protocol default of three 200 ms attempts for power-up.
            'feedback_read_retries': 1,
            'feedback_read_timeout_sec': 0.05,
            # USB enumeration and MD200T power-up do not finish at exactly
            # the same time.  Communication loss stops output and reuses this
            # retry path; motor state/current/overspeed faults remain latched.
            'startup_retry_interval_sec': 2.0,
            'startup_retry_log_interval_sec': 10.0,
            # Commanded speed and measured overspeed are separate limits: an
            # unloaded wheel may temporarily spin faster than the other three.
            'feedback_overspeed_limit_rpm': 90.0,
            'abort_current_a': 9.5,
            # HOLD is an armed zero-velocity state.  It intentionally does not
            # send PID100 stop/torque-off, so the MD200T velocity loop can
            # resist motion while the manipulator is operating.
            'hold_maximum_rpm': 2.0,
            'hold_stable_sec': 0.50,
            # Variances for a planar skid-steer wheel estimate. Unobserved
            # dimensions are intentionally large so fusion does not treat the
            # wheel controller as a 3D pose sensor.
            'pose_covariance_diagonal': [0.04, 0.04, 1.0e6,
                                         1.0e6, 1.0e6, 0.09],
            'twist_covariance_diagonal': [0.01, 1.0e6, 1.0e6,
                                          1.0e6, 1.0e6, 0.04],
            # Lifted-wheel commissioning verified physical forward as
            # MOT1 positive and MOT2 negative on both controllers.
            'left_mot1_command_sign': 1,
            'left_mot2_command_sign': -1,
            'right_mot1_command_sign': 1,
            'right_mot2_command_sign': -1,
            # Normalize electrical shaft RPM into physical wheel-forward RPM.
            'left_mot1_feedback_sign': 1,
            'left_mot2_feedback_sign': -1,
            'right_mot1_feedback_sign': 1,
            'right_mot2_feedback_sign': -1,
            'publish_tf': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def p(name):
            return self.get_parameter(name).value
        self.simulation = p('simulation_mode')
        self.allow_motion = p('allow_motion')
        self.layout = p('hardware_layout')
        self.single_side = p('single_side')
        self.radius = float(p('wheel_radius'))
        self.separation = float(p('wheel_separation'))
        self.max_linear = float(p('max_linear_speed'))
        self.max_angular = float(p('max_angular_speed'))
        self.angular_command_sign = int(p('angular_command_sign'))
        self.max_rpm = float(p('max_wheel_rpm'))
        self.timeout = float(p('command_timeout_sec'))
        self.rate = float(p('command_rate_hz'))
        self.feedback_rate = float(p('feedback_rate_hz'))
        self.feedback_timeout = float(p('feedback_timeout_sec'))
        self.feedback_read_retries = int(p('feedback_read_retries'))
        self.feedback_read_timeout = float(p('feedback_read_timeout_sec'))
        self.startup_retry_interval = float(p('startup_retry_interval_sec'))
        self.startup_retry_log_interval = float(
            p('startup_retry_log_interval_sec'))
        self.feedback_overspeed_limit = float(
            p('feedback_overspeed_limit_rpm'))
        self.abort_current = float(p('abort_current_a'))
        self.hold_maximum_rpm = float(p('hold_maximum_rpm'))
        self.hold_stable_sec = float(p('hold_stable_sec'))
        self.pose_covariance = diagonal_covariance(
            p('pose_covariance_diagonal'))
        self.twist_covariance = diagonal_covariance(
            p('twist_covariance_diagonal'))
        self.publish_tf_enabled = p('publish_tf')
        self.odom_frame = p('odom_frame_id')
        self.base_frame = p('base_frame_id')
        self.command_signs = {
            'left': (p('left_mot1_command_sign'), p('left_mot2_command_sign')),
            'right': (p('right_mot1_command_sign'), p('right_mot2_command_sign')),
        }
        self.feedback_signs = {
            'left': (p('left_mot1_feedback_sign'), p('left_mot2_feedback_sign')),
            'right': (p('right_mot1_feedback_sign'), p('right_mot2_feedback_sign')),
        }
        self._validate_parameters()

        self.controllers = {}
        self.feedback = {}
        self.last_feedback_time = {}
        self.fault = False
        self.cmd = Twist()
        self.last_cmd_time = None
        self.last_update = time.monotonic()
        self.feedback_tick = 0
        self.next_startup_attempt = 0.0
        self.last_startup_log = 0.0
        self.last_startup_error = ''
        self.last_feedback_miss_log = {}
        self.hold_requested = False
        self.hold_ready = False
        self.hold_stable_since = None
        self.x = self.y = self.yaw = 0.0
        self.wheel_position = {'left': 0.0, 'right': 0.0}

        self.connected_pub = self.create_publisher(
            Bool, '/md/base_connected', 1)
        self.hold_active_pub = self.create_publisher(Bool, '/md/hold_active', 1)
        self.hold_ready_pub = self.create_publisher(Bool, '/md/hold_ready', 1)
        self.raw_pubs = {
            side: self.create_publisher(
                Float64MultiArray, f'/md/{side}/raw_state', 10)
            for side in ('left', 'right')
        }
        self.commanded_rpm_pub = self.create_publisher(
            Float64MultiArray, '/md/commanded_rpm', 10)
        self.odom_pub = self.create_publisher(Odometry, p('odom_topic'), 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, p('cmd_vel_topic'), self._on_cmd, 10)
        self.create_service(SetBool, '/md/set_hold', self._set_hold)

        self.create_timer(1.0 / self.rate, self._update)
        if self.simulation:
            self._log_driver_ready()
        else:
            self._attempt_startup_connection(force=True)

    def _validate_parameters(self):
        if self.layout not in ('single', 'dual'):
            raise ValueError('hardware_layout must be single or dual')
        if self.single_side not in ('left', 'right'):
            raise ValueError('single_side must be left or right')
        if min(self.radius, self.separation, self.max_linear,
               self.max_angular, self.max_rpm, self.timeout, self.rate,
               self.feedback_timeout, self.feedback_read_timeout,
               self.startup_retry_interval,
               self.startup_retry_log_interval) <= 0:
            raise ValueError('geometry, limits, timeout, and rate must be positive')
        if self.feedback_read_retries < 1:
            raise ValueError('feedback_read_retries must be at least one')
        if not 0 < self.feedback_rate <= self.rate:
            raise ValueError('feedback_rate_hz must be in (0, command_rate_hz]')
        if self.abort_current <= 0:
            raise ValueError('abort_current_a must be positive')
        if self.hold_maximum_rpm < 0.0 or self.hold_stable_sec <= 0.0:
            raise ValueError('hold limits must be non-negative/positive')
        if self.feedback_overspeed_limit <= self.max_rpm:
            raise ValueError(
                'feedback_overspeed_limit_rpm must exceed max_wheel_rpm')
        if self.angular_command_sign not in (-1, 1):
            raise ValueError('angular_command_sign must be -1 or 1')
        sign_pairs = list(self.command_signs.values()) + list(
            self.feedback_signs.values())
        if any(sign not in (-1, 1) for pair in sign_pairs for sign in pair):
            raise ValueError('motor direction signs must be -1 or 1')

    def _active_sides(self):
        return ('left', 'right') if self.layout == 'dual' else (self.single_side,)

    def _hardware_open(self):
        return len(self.controllers) == len(self._active_sides())

    def _log_driver_ready(self):
        self.get_logger().info(
            f'MD200T driver ready: simulation={self.simulation}, '
            f'layout={self.layout}, speed_limit={self.max_linear:.2f} m/s, '
            f'command_limit={self.max_rpm:.0f} RPM, '
            f'feedback_guard={self.feedback_overspeed_limit:.0f} RPM, '
            f'current_guard={self.abort_current:.1f} A')

    def _attempt_startup_connection(self, force=False):
        if self.simulation or self.fault or self._hardware_open():
            return self._hardware_open()
        now = time.monotonic()
        if not force and now < self.next_startup_attempt:
            return False
        self.next_startup_attempt = now + self.startup_retry_interval
        try:
            self._open_hardware()
        except MotorSafetyFault as error:
            self._lock_fault(str(error))
            self.connected_pub.publish(Bool(data=False))
            return False
        except (OSError, RuntimeError, ValueError) as error:
            detail = str(error)
            should_log = (
                detail != self.last_startup_error or
                now - self.last_startup_log >= self.startup_retry_log_interval
            )
            if should_log:
                self.get_logger().warning(
                    f'MD200T not ready: {detail}; motion remains locked, '
                    f'automatic retry in {self.startup_retry_interval:.1f} s')
                self.last_startup_log = now
                self.last_startup_error = detail
            self.connected_pub.publish(Bool(data=False))
            return False
        self.feedback_tick = 0
        self.cmd = Twist()
        self.last_cmd_time = None
        self.last_startup_error = ''
        self.get_logger().info('MD200T PID210 connected on all configured sides.')
        if not self.allow_motion:
            self.get_logger().warning(
                'MD200T connected, but non-zero motion is locked '
                '(allow_motion=false).')
        self._log_driver_ready()
        return True

    def _open_hardware(self):
        baud = self.get_parameter('baudrate').value
        try:
            for side in self._active_sides():
                port = self.get_parameter(f'{side}_serial_port').value
                controller = Md200tSerial(port, baud)
                controller.open()
                # Register the open descriptor before the first PID210 read.
                # If power-up feedback is late, the shared exception cleanup
                # must still close this port so the automatic retry cannot be
                # blocked by our own leaked flock.
                self.controllers[side] = controller
                feedback = controller.read_feedback()
                self._check_feedback(side, feedback, require_stopped=True)
                self.feedback[side] = feedback
                self.last_feedback_time[side] = time.monotonic()
            self._stop_hardware()
            if self.allow_motion:
                for controller in self.controllers.values():
                    controller.arm_serial(start_mode=1)
                    controller.command_channel(1, 0)
                    controller.command_channel(2, 0)
        except BaseException:
            self._close_hardware()
            raise

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def _set_hold(self, request, response):
        requested = bool(request.data)
        if requested and not self.allow_motion:
            response.success = False
            response.message = 'HOLD requires allow_motion=true'
            return response
        if requested and self.fault:
            response.success = False
            response.message = 'HOLD rejected: MD200T has a latched fault'
            return response
        if requested and not (self.simulation or self._hardware_open()):
            response.success = False
            response.message = 'HOLD rejected: MD200T feedback is not connected'
            return response

        self.hold_requested = requested
        self.hold_ready = False
        self.hold_stable_since = None
        # Discard a command that may have arrived immediately before HOLD.
        # Releasing HOLD also requires a new, fresh command from the owner.
        self.cmd = Twist()
        self.last_cmd_time = None
        response.success = True
        response.message = (
            '[MD-HOLD] MD200T zero-RPM HOLD requested'
            if requested else '[MD-HOLD] MD200T zero-RPM HOLD released'
        )
        self.get_logger().info(response.message)
        return response

    def _update_hold_state(self, now, connected, commands):
        active = bool(
            self.hold_requested and connected and self.allow_motion and not self.fault
        )
        stopped = False
        if active:
            if self.simulation:
                stopped = all(abs(value) <= self.hold_maximum_rpm for value in commands.values())
            else:
                stopped = hold_feedback_stopped(self.feedback, self.hold_maximum_rpm)
        if active and stopped:
            if self.hold_stable_since is None:
                self.hold_stable_since = now
            self.hold_ready = now - self.hold_stable_since >= self.hold_stable_sec
        else:
            self.hold_stable_since = None
            self.hold_ready = False
        self.hold_active_pub.publish(Bool(data=active))
        self.hold_ready_pub.publish(Bool(data=self.hold_ready))

    def _update(self):
        now = time.monotonic()
        if not self.simulation and not self._hardware_open():
            if not self._attempt_startup_connection():
                # Do not publish wheel odometry/TF before real PID210 feedback;
                # the field preflight must remain NOT READY and fail closed.
                self.connected_pub.publish(Bool(data=False))
                self.last_update = now
                return
        fresh = (
            self.last_cmd_time is not None and
            now - self.last_cmd_time <= self.timeout)
        enabled = (
            (fresh or self.hold_requested) and (self.simulation or self.allow_motion) and
            not self.fault and (self.simulation or self._hardware_open())
        )
        linear = (
            max(-self.max_linear, min(self.max_linear, self.cmd.linear.x))
            if enabled else 0.0)
        angular = (
            self.angular_command_sign * max(
                -self.max_angular,
                min(self.max_angular, self.cmd.angular.z))
            if enabled else 0.0)
        if self.hold_requested:
            linear = 0.0
            angular = 0.0
        if self.layout == 'single':
            # A one-sided chassis must not be steered. The lifted-wheel test uses
            # linear.x only and sends that side's physical-forward wheel speed.
            angular = 0.0
        left_rpm, right_rpm = twist_to_wheel_rpm(
            linear, angular, self.separation, self.radius, self.max_rpm)
        commands = {'left': left_rpm, 'right': right_rpm}
        command_message = Float64MultiArray()
        command_message.data = [
            float(commands.get('left', 0.0)),
            float(commands.get('right', 0.0)),
        ]
        self.commanded_rpm_pub.publish(command_message)

        newly_read_feedback = set()
        if not self.simulation and not self.fault:
            self.feedback_tick += 1
            interval = max(1, int(round(self.rate / self.feedback_rate)))
            request_feedback = self.feedback_tick >= interval
            if request_feedback:
                self.feedback_tick = 0
            try:
                for side, controller in self.controllers.items():
                    if self.allow_motion:
                        rpm = commands[side]
                        signs = self.command_signs[side]
                        controller.command_channel(
                            1, int(round(rpm * signs[0])))
                        controller.command_channel(
                            2, int(round(rpm * signs[1])))
                    if request_feedback:
                        try:
                            result = controller.read_feedback(
                                retries=self.feedback_read_retries,
                                timeout=self.feedback_read_timeout)
                        except RuntimeError as error:
                            if not is_transient_feedback_timeout(error):
                                raise
                            # PID210 is sampled at feedback_rate_hz.  Preserve
                            # the last valid sample for the configured freshness
                            # window instead of latching a fault on one missed
                            # USB/RS-485 reply.  The freshness check below stops
                            # output and starts automatic reconnect if replies
                            # remain absent.
                            last_log = self.last_feedback_miss_log.get(
                                side, 0.0)
                            current = time.monotonic()
                            if current - last_log >= self.feedback_timeout:
                                age = current - self.last_feedback_time.get(
                                    side, current)
                                self.get_logger().warning(
                                    f'{side} PID210 sample missed; retaining '
                                    f'last valid feedback (age={age:.2f} s, '
                                    f'reconnect at {self.feedback_timeout:.2f} s)')
                                self.last_feedback_miss_log[side] = current
                            continue
                        self._check_feedback(side, result)
                        self.feedback[side] = result
                        self.last_feedback_time[side] = time.monotonic()
                        newly_read_feedback.add(side)
            except MotorSafetyFault as error:
                self._lock_fault(str(error))
            except (OSError, RuntimeError, ValueError) as error:
                self._begin_communication_recovery(str(error))
        elif self.simulation:
            self._publish_simulated_raw(commands)

        hardware_connected = self._hardware_feedback_fresh(time.monotonic())
        if (not self.simulation and not self.fault and self._hardware_open()
                and not hardware_connected):
            self._begin_communication_recovery(
                f'PID210 feedback stale for more than '
                f'{self.feedback_timeout:.2f} s')
            hardware_connected = False
        connected = self.simulation or (hardware_connected and not self.fault)
        self.connected_pub.publish(Bool(data=connected))
        self._update_hold_state(time.monotonic(), connected, commands)
        self._publish_raw_feedback(
            set(self._active_sides()) if self.simulation else newly_read_feedback)
        dt = min(0.2, max(0.0, now - self.last_update))
        self.last_update = now
        if self.layout == 'dual':
            odom_rpm = commands
            if not self.simulation and connected and all(
                    side in self.feedback for side in ('left', 'right')):
                odom_rpm = {
                    side: self._physical_feedback_rpm(side)
                    for side in ('left', 'right')
                }
            elif not connected:
                odom_rpm = {'left': 0.0, 'right': 0.0}
            self._publish_odometry(
                odom_rpm['left'], odom_rpm['right'], dt)

    def _physical_feedback_rpm(self, side):
        signs = self.feedback_signs[side]
        feedback = self.feedback[side]
        return 0.5 * (
            feedback[1].rpm * signs[0] + feedback[2].rpm * signs[1])

    def _hardware_feedback_fresh(self, now):
        if self.simulation:
            return True
        active = self._active_sides()
        return (
            len(self.controllers) == len(active)
            and all(
                side in self.last_feedback_time
                and now - self.last_feedback_time[side] <= self.feedback_timeout
                for side in active
            )
        )

    def _check_feedback(self, side, feedback, require_stopped=False):
        for channel, motor in feedback.items():
            if motor.state != 0:
                raise MotorSafetyFault(
                    f'{side} MOT{channel} state={motor.state}')
            if motor.current_a > self.abort_current:
                raise MotorSafetyFault(
                    f'{side} MOT{channel} current {motor.current_a:.1f} A exceeds limit')
            if abs(motor.rpm) > self.feedback_overspeed_limit:
                raise MotorSafetyFault(
                    f'{side} MOT{channel} overspeed {motor.rpm} RPM exceeds '
                    f'{self.feedback_overspeed_limit:.0f} RPM feedback limit')
            if require_stopped and abs(motor.rpm) > 2:
                raise MotorSafetyFault(
                    f'{side} MOT{channel} is moving at startup')

    def _publish_simulated_raw(self, commands):
        # Raw simulation is emitted directly below; no fake hardware object is needed.
        self.sim_commands = commands

    def _publish_raw_feedback(self, sides):
        for side in sides:
            msg = Float64MultiArray()
            if self.simulation:
                physical = getattr(self, 'sim_commands', {}).get(side, 0.0)
                msg.data = [physical, physical, 0.0, 0.0, 0.0, 0.0]
            elif side in self.feedback:
                first, second = self.feedback[side][1], self.feedback[side][2]
                msg.data = [float(first.rpm), float(second.rpm),
                            first.current_a, second.current_a,
                            float(first.state), float(second.state)]
            else:
                continue
            self.raw_pubs[side].publish(msg)

    def _publish_odometry(self, left_rpm, right_rpm, dt):
        left_mps, right_mps = rpm_to_mps(left_rpm, self.radius), rpm_to_mps(right_rpm, self.radius)
        linear = (left_mps + right_mps) * 0.5
        angular = (right_mps - left_mps) / self.separation
        self.yaw += angular * dt
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = linear, angular
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance
        self.odom_pub.publish(odom)
        if self.publish_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id = stamp, self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
            transform.transform.rotation.z = math.sin(self.yaw * 0.5)
            transform.transform.rotation.w = math.cos(self.yaw * 0.5)
            self.tf_broadcaster.sendTransform(transform)
        self.wheel_position['left'] += left_rpm * 2 * math.pi / 60 * dt
        self.wheel_position['right'] += right_rpm * 2 * math.pi / 60 * dt
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['left_wheel_joint', 'right_wheel_joint']
        joints.position = [self.wheel_position['left'], self.wheel_position['right']]
        joints.velocity = [left_rpm * 2 * math.pi / 60, right_rpm * 2 * math.pi / 60]
        self.joint_pub.publish(joints)

    def _stop_hardware(self):
        for controller in self.controllers.values():
            controller.stop()

    def _lock_fault(self, reason):
        self.fault = True
        self.hold_ready = False
        self.hold_stable_since = None
        self.get_logger().error(f'MD200T output locked: {reason}')
        try:
            self._stop_hardware()
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f'emergency stop failed: {error}')

    def _begin_communication_recovery(self, reason):
        """Stop output and reopen serial links without latching a motor fault."""
        self.get_logger().warning(
            f'MD200T communication interrupted: {reason}; output stopped, '
            f'automatic reconnect in {self.startup_retry_interval:.1f} s')
        self.cmd = Twist()
        self.last_cmd_time = None
        self.feedback_tick = 0
        self.hold_ready = False
        self.hold_stable_since = None
        self.connected_pub.publish(Bool(data=False))
        self._close_hardware()
        self.last_feedback_time.clear()
        self.next_startup_attempt = (
            time.monotonic() + self.startup_retry_interval)

    def _close_hardware(self):
        for controller in self.controllers.values():
            try:
                controller.close()
            except (OSError, RuntimeError, ValueError) as error:
                self.get_logger().error(f'close/stop failed: {error}')
        self.controllers.clear()

    def destroy_node(self):
        self._close_hardware()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = ROKAMdDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (OSError, RuntimeError, ValueError) as error:
        # Startup failures are summarized once.  The integrated field checker
        # reports which adapter/feedback is missing without a Python traceback.
        rclpy.logging.get_logger('roka_md_driver').error(
            f'MD200T startup failed: {error}')
        return 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ros2 launch forwards SIGINT to children during an orderly
                # shutdown. A second signal while closing serial ports must
                # not turn a clean operator stop into a Python traceback.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0
