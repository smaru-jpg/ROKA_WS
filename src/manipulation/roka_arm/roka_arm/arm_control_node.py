#!/usr/bin/env python3
"""ROKA arm controller with an explicit no-motion startup gate."""

from hashlib import sha256
from pathlib import Path
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from roka_interfaces.msg import BoxObservation
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from . import arm_config as cfg
from .grasp_sequence import GraspSequence
from .manual_arm_joy import ManualTargets, integrate_targets
from .visual_servo import VisualServoController


class NoMotionDriver:
    """Read/write facade used by camera-only observation mode."""

    motion_enabled = False

    def __getattr__(self, _name):
        return lambda *_args, **_kwargs: False

    def close_ports(self):
        return None


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("roka_arm_controller")
        values = self._declare_and_read_parameters()
        cfg.apply_runtime_overrides(values)
        self.start_pose_name = str(values["start_pose_name"])
        self.start_pose = cfg.activate_start_pose(self.start_pose_name)
        self.perception_enabled = bool(values["perception_enabled"])
        self.manual_joy_enabled = bool(values["manual_joy_enabled"])
        if self.manual_joy_enabled and not cfg.ENABLE_MOTION:
            raise RuntimeError("manual arm control requires enable_motion=true")
        if self.perception_enabled:
            self._validate_model()

        # Model/camera software is validated before any actuator port is opened.
        if self.perception_enabled:
            from .perception import Perception

            self.perception = Perception()
        else:
            self.perception = None
        self.driver = self._initialize_driver()
        self.visual_servo = VisualServoController(self.driver)
        self.grasp = GraspSequence(self.driver)

        self.auto_control = cfg.AUTO_CONTROL_START and cfg.ENABLE_MOTION
        self.ready_stable_count = 0
        self.ready_pose = None
        self.grasp_sequence_started = False
        self.grasp_sequence_finished = False
        self.grasp_final_failed = False
        self.grasp_attempt_count = 0
        self.last_inference_time = 0.0
        self._target_geometry_validated = False
        self._grasp_thread = None
        self.named_pose_active = False
        self.named_pose_failed = False
        self.named_pose_name = ""
        self._pose_thread = None
        self.manual_faulted = False
        self.latest_joy = None
        self.latest_joy_time = 0.0
        self.last_manual_update_time = time.monotonic()
        self.manual_targets = ManualTargets(
            joint_13=int(self.start_pose[cfg.JOINT_3_ID]),
            joint_11=int(self.start_pose[cfg.JOINT_1_ID]),
            wrist_4=int(self.start_pose["wrist"]),
        )

        if self.perception is not None:
            self.image_sub = self.create_subscription(
                Image,
                cfg.CAMERA_TOPIC,
                self.image_callback,
                qos_profile_sensor_data,
            )
            self.depth_sub = self.create_subscription(
                Image,
                cfg.DEPTH_TOPIC,
                self.depth_callback,
                qos_profile_sensor_data,
            )
        self.status_pub = self.create_publisher(String, "/roka/arm/status", 1)
        hold_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.hold_ready_pub = self.create_publisher(
            Bool, "/roka/arm/hold_ready", hold_qos
        )
        self.observation_pub = self.create_publisher(
            BoxObservation,
            "/roka/arm/box_observation",
            qos_profile_sensor_data,
        )
        self.auto_service = self.create_service(
            SetBool,
            "/roka/arm/set_auto_control",
            self.set_auto_control,
        )
        self.perception_service = self.create_service(
            SetBool,
            "/roka/arm/set_perception_enabled",
            self.set_perception_enabled,
        )
        self.pose_services = []
        for service_name, pose_name in (
            ("/roka/arm/pose/section1", "section1"),
            ("/roka/arm/pose/section2_start", "section2_start"),
            ("/roka/arm/pose/section2_drive", "section2_drive"),
        ):
            self.pose_services.append(self.create_service(
                Trigger,
                service_name,
                lambda request, response, selected=pose_name: self._request_named_pose(
                    selected, request, response
                ),
            ))
        if self.manual_joy_enabled:
            self.create_subscription(
                Joy, str(values["joy_topic"]), self._manual_joy, 20
            )
            self.create_timer(
                1.0 / float(values["manual_control_rate_hz"]),
                self._manual_control,
            )
        self.create_timer(1.0, self.publish_status)
        self._publish_hold_ready(bool(cfg.ENABLE_MOTION) and not self.auto_control)

        mode = "MOTION" if cfg.ENABLE_MOTION else "OBSERVE"
        perception_detail = (
            f"model={Path(cfg.YOLO_MODEL_PATH).name}, "
            f"device={self.perception.inference_device}, "
            f"rate={cfg.INFERENCE_RATE_HZ:.1f}Hz"
            if self.perception is not None
            else "box_perception=disabled"
        )
        self.get_logger().info(
            f"ready mode={mode}, {perception_detail}, "
            f"start_pose={self.start_pose_name}, manual_joy={self.manual_joy_enabled}"
        )

    def _declare_and_read_parameters(self):
        defaults = {
            "arm_serial_port": cfg.DEVICENAME_ARM,
            "wrist_serial_port": cfg.DEVICENAME_WRIST,
            "connect_hardware": cfg.CONNECT_HARDWARE,
            "enable_motion": cfg.ENABLE_MOTION,
            "torque_off_on_exit": cfg.TORQUE_OFF_ON_EXIT,
            "color_topic": cfg.CAMERA_TOPIC,
            "depth_topic": cfg.DEPTH_TOPIC,
            "model_path": cfg.YOLO_MODEL_PATH,
            "visualize": cfg.VISUALIZE,
            "inference_device": cfg.INFERENCE_DEVICE,
            "inference_rate_hz": cfg.INFERENCE_RATE_HZ,
            "maximum_depth_age_sec": cfg.MAXIMUM_DEPTH_AGE_SEC,
            "target_class_ids": list(cfg.TARGET_CLASS_IDS),
            "activation_min_y_ratio": cfg.ACTIVATION_MIN_Y_RATIO,
            "auto_control_start": cfg.AUTO_CONTROL_START,
            "target_x": cfg.TARGET_X,
            "target_y": cfg.TARGET_Y,
            "target_depth_mm": cfg.TARGET_DEPTH_MM,
            "y_delta": cfg.Y_DELTA,
            "x_command_interval": cfg.X_COMMAND_INTERVAL,
            "y_command_interval": cfg.Y_COMMAND_INTERVAL,
            "z_command_interval": cfg.Z_COMMAND_INTERVAL,
            "start_pose_name": f"section{cfg.DEFAULT_SECTION}",
            "perception_enabled": True,
            "manual_joy_enabled": False,
            "joy_topic": "/joy",
            "manual_enable_button": 5,
            "manual_joint_13_axis": 4,
            "manual_joint_11_axis": 3,
            "manual_wrist_axis": 7,
            "manual_deadzone": 0.18,
            "manual_control_rate_hz": 20.0,
            "manual_joy_timeout_sec": 0.35,
            "manual_joint_13_rate_counts_sec": 8000.0,
            "manual_joint_11_rate_counts_sec": 5000.0,
            "manual_wrist_rate_counts_sec": 300.0,
            "manual_joint_13_sign": 1.0,
            "manual_joint_11_sign": 1.0,
            "manual_wrist_sign": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        return {name: self.get_parameter(name).value for name in defaults}

    def _validate_model(self):
        model = Path(cfg.YOLO_MODEL_PATH)
        if not model.is_file():
            raise RuntimeError(f"YOLO model does not exist: {model}")
        digest = sha256(model.read_bytes()).hexdigest()
        if digest != cfg.YOLO_MODEL_SHA256:
            raise RuntimeError(
                f"YOLO model checksum mismatch: expected "
                f"{cfg.YOLO_MODEL_SHA256}, got {digest}"
            )

    def _initialize_driver(self):
        if not cfg.CONNECT_HARDWARE:
            if cfg.ENABLE_MOTION:
                raise RuntimeError("motion cannot be enabled without hardware")
            return NoMotionDriver()

        try:
            from .dynamixel_driver import DynamixelDriver
        except Exception as error:
            raise RuntimeError(
                "Dynamixel Python runtime is unavailable; install python3-serial"
            ) from error

        driver = DynamixelDriver()
        if not driver.initialize(
            enable_motion=cfg.ENABLE_MOTION,
            allow_initial_recovery=cfg.ENABLE_MOTION,
        ):
            driver.close_ports()
            raise RuntimeError("Dynamixel initialization failed")
        if cfg.ENABLE_MOTION:
            if not driver.wait_until_pose_values_reached(
                "SECTION START HOLD", self.start_pose
            ):
                driver.close_ports()
                raise RuntimeError("arm did not reach the selected section start pose")
            if not driver.all_torque_enabled():
                driver.close_ports()
                raise RuntimeError("one or more Dynamixel torque locks are not enabled")
        return driver

    def set_auto_control(self, request, response):
        if request.data and not cfg.ENABLE_MOTION:
            response.success = False
            response.message = "motion is disabled"
            return response
        if request.data and self.grasp_sequence_started:
            response.success = False
            response.message = "grasp sequence is running"
            return response
        if request.data and self.perception is None:
            response.success = False
            response.message = "box perception is not loaded in this section"
            return response
        if request.data and self.named_pose_active:
            response.success = False
            response.message = "named pose movement is running"
            return response
        self.auto_control = bool(request.data)
        # While visual servoing/grasping, the MD coordinator independently
        # holds the base at zero RPM. Once disabled, the arm remains torque-
        # held in its current transport/carry pose for resumed driving.
        self._publish_hold_ready(cfg.ENABLE_MOTION and not self.auto_control)
        response.success = True
        response.message = "auto control ON" if self.auto_control else "auto control OFF"
        self.get_logger().info(response.message)
        return response

    def set_perception_enabled(self, request, response):
        if request.data and self.perception is None:
            response.success = False
            response.message = "box perception is not loaded in this section"
            return response
        if request.data and self.grasp_sequence_finished:
            response.success = False
            response.message = "box mission already completed"
            return response
        self.perception_enabled = bool(request.data)
        if not self.perception_enabled:
            self.auto_control = False
            self.ready_stable_count = 0
            self._publish_hold_ready(cfg.ENABLE_MOTION)
            if cfg.VISUALIZE:
                try:
                    cv2.destroyWindow(cfg.WINDOW_NAME)
                except cv2.error:
                    pass
        response.success = True
        response.message = (
            "box perception ON" if self.perception_enabled else "box perception OFF"
        )
        self.get_logger().info(response.message)
        return response

    def _request_named_pose(self, pose_name, _request, response):
        if not cfg.ENABLE_MOTION:
            response.success = False
            response.message = "motion is disabled"
            return response
        if self.auto_control or self.grasp_sequence_started or self.named_pose_active:
            response.success = False
            response.message = "arm is busy"
            return response
        try:
            if pose_name == "section2_drive":
                pose = dict(cfg.SECTION2_DRIVE_POSE)
            else:
                pose = cfg.get_named_start_pose(pose_name)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        self.named_pose_active = True
        self.named_pose_failed = False
        self.named_pose_name = pose_name
        self._publish_hold_ready(False)
        self._pose_thread = threading.Thread(
            target=self._run_named_pose,
            args=(pose_name, pose),
            name=f"roka-arm-pose-{pose_name}",
            daemon=True,
        )
        self._pose_thread.start()
        response.success = True
        response.message = f"{pose_name} movement accepted"
        return response

    def _run_named_pose(self, pose_name, pose):
        ok = False
        try:
            ok = self.driver.move_pose_values(pose_name.upper(), pose)
            if ok:
                ok = self.driver.wait_until_pose_values_reached(
                    pose_name.upper(), pose
                )
            if ok:
                self._set_manual_targets_from_pose(pose)
                self.get_logger().info(f"named pose reached: {pose_name}")
            else:
                self.get_logger().error(f"named pose failed: {pose_name}")
        except Exception as error:
            self.get_logger().error(f"named pose exception ({pose_name}): {error}")
            ok = False
        finally:
            self.named_pose_failed = not ok
            self.named_pose_active = False
            self._publish_hold_ready(bool(ok) and cfg.ENABLE_MOTION)

    def _set_manual_targets_from_pose(self, pose):
        self.manual_targets = ManualTargets(
            joint_13=int(pose[cfg.JOINT_3_ID]),
            joint_11=int(pose[cfg.JOINT_1_ID]),
            wrist_4=int(pose["wrist"]),
        )

    def _manual_joy(self, message):
        self.latest_joy = message
        self.latest_joy_time = time.monotonic()

    @staticmethod
    def _joy_axis(message, index):
        return float(message.axes[index]) if 0 <= index < len(message.axes) else 0.0

    def _manual_control(self):
        now = time.monotonic()
        dt = now - self.last_manual_update_time
        self.last_manual_update_time = now
        message = self.latest_joy
        enable_button = int(self.get_parameter("manual_enable_button").value)
        enabled = (
            message is not None
            and 0 <= enable_button < len(message.buttons)
            and bool(message.buttons[enable_button])
        )
        if (
            self.manual_faulted
            or not enabled
            or now - self.latest_joy_time
            > float(self.get_parameter("manual_joy_timeout_sec").value)
            or self.auto_control
            or self.grasp_sequence_started
            or self.named_pose_active
        ):
            return
        updated = integrate_targets(
            self.manual_targets,
            joint_13_axis=self._joy_axis(
                message, int(self.get_parameter("manual_joint_13_axis").value)
            ),
            joint_11_axis=self._joy_axis(
                message, int(self.get_parameter("manual_joint_11_axis").value)
            ),
            wrist_axis=self._joy_axis(
                message, int(self.get_parameter("manual_wrist_axis").value)
            ),
            dt=dt,
            deadzone=float(self.get_parameter("manual_deadzone").value),
            joint_13_rate=float(
                self.get_parameter("manual_joint_13_rate_counts_sec").value
            ),
            joint_11_rate=float(
                self.get_parameter("manual_joint_11_rate_counts_sec").value
            ),
            wrist_rate=float(
                self.get_parameter("manual_wrist_rate_counts_sec").value
            ),
            joint_13_sign=float(self.get_parameter("manual_joint_13_sign").value),
            joint_11_sign=float(self.get_parameter("manual_joint_11_sign").value),
            wrist_sign=float(self.get_parameter("manual_wrist_sign").value),
        )
        ok = True
        if updated.joint_13 != self.manual_targets.joint_13:
            ok = self.driver.write_arm_position(
                cfg.JOINT_3_ID, updated.joint_13
            ) and ok
        if updated.joint_11 != self.manual_targets.joint_11:
            ok = self.driver.write_arm_position(
                cfg.JOINT_1_ID, updated.joint_11
            ) and ok
        if updated.wrist_4 != self.manual_targets.wrist_4:
            ok = self.driver.write_wrist_position(updated.wrist_4) and ok
        if ok:
            self.manual_targets = updated
        else:
            self.manual_faulted = True
            self._publish_hold_ready(False)
            self.get_logger().error("manual arm command failed; base remains locked")

    def _publish_hold_ready(self, ready):
        message = Bool()
        message.data = bool(ready)
        self.hold_ready_pub.publish(message)

    def publish_status(self):
        message = String()
        if self.named_pose_failed or self.manual_faulted:
            state = "ARM_FAILED"
        elif self.named_pose_active:
            state = "POSE_MOVING"
        elif self.grasp_final_failed:
            state = "GRASP_FAILED"
        elif self.grasp_sequence_finished:
            state = "GRASP_COMPLETE"
        elif self.grasp_sequence_started:
            state = "GRASP_RUNNING"
        elif self.auto_control:
            state = "AUTO"
        else:
            state = "OBSERVE"
        message.data = (
            f"state={state};hardware={cfg.CONNECT_HARDWARE};"
            f"motion={cfg.ENABLE_MOTION};perception={self.perception_enabled};"
            f"start_pose={self.start_pose_name}"
        )
        self.status_pub.publish(message)

    def depth_callback(self, msg):
        if self.perception is not None:
            self.perception.update_depth(msg)

    def _publish_observation(self, header, frame, detection, depth_mm):
        observation = BoxObservation()
        observation.header = header
        observation.detected = detection is not None
        if detection is not None:
            height, width = frame.shape[:2]
            observation.class_id = int(detection["class_id"])
            observation.class_name = str(detection["class_name"])
            observation.confidence = float(detection["conf"])
            observation.center_x_ratio = float(detection["cx"]) / float(width)
            observation.center_y_ratio = float(detection["cy"]) / float(height)
            observation.depth_valid = depth_mm is not None
            observation.depth_mm = float(depth_mm or 0.0)
            observation.activation_roi = (
                observation.center_y_ratio >= cfg.ACTIVATION_MIN_Y_RATIO
            )
        self.observation_pub.publish(observation)

    def _validate_target_geometry(self, frame):
        if self._target_geometry_validated:
            return True
        height, width = frame.shape[:2]
        if not (0 <= cfg.TARGET_X < width and 0 <= cfg.TARGET_Y < height):
            self.auto_control = False
            self.get_logger().error(
                f"target pixel ({cfg.TARGET_X},{cfg.TARGET_Y}) is outside "
                f"the camera frame {width}x{height}"
            )
            return False
        self._target_geometry_validated = True
        self.get_logger().info(
            f"camera geometry accepted: {width}x{height}, "
            f"target=({cfg.TARGET_X},{cfg.TARGET_Y})"
        )
        return True

    def _start_grasp(self):
        if self.grasp_sequence_started:
            return "GRASP RUNNING"
        ready_pose = self.driver.read_current_pose()
        if not ready_pose:
            return "READY READ ERROR"
        poses = self.grasp.make_grasp_poses(ready_pose)
        if poses is None:
            return "READY CONFIG ERROR"

        self.ready_pose = ready_pose
        self.auto_control = False
        self._publish_hold_ready(False)
        self.grasp_sequence_started = True
        self.grasp_attempt_count += 1
        self._grasp_thread = threading.Thread(
            target=self._run_grasp,
            args=(poses,),
            name="roka-arm-grasp",
            daemon=True,
        )
        self._grasp_thread.start()
        return "GRASP STARTED"

    def _run_grasp(self, poses):
        try:
            if not self.grasp.run(poses):
                self.grasp_final_failed = True
                return
            time.sleep(cfg.GRASP_VERIFY_DELAY)
            samples = []
            for _ in range(cfg.GRASP_CURRENT_SAMPLES):
                current = self.driver.read_gripper_current_mA()
                if current is not None:
                    samples.append(abs(current))
                time.sleep(cfg.GRASP_CURRENT_SAMPLE_INTERVAL)
            average = sum(samples) / len(samples) if samples else 0.0
            if samples and average >= cfg.GRASP_CURRENT_THRESHOLD:
                if not self.grasp.run_post_grasp(poses):
                    self.grasp_final_failed = True
                    self.get_logger().error(
                        "grasp succeeded but place/drop/drive sequence failed"
                    )
                    return
                if "SECTION2_DRIVE" in poses and hasattr(self, "manual_targets"):
                    self._set_manual_targets_from_pose(poses["SECTION2_DRIVE"])
                self.grasp_sequence_finished = True
                self.get_logger().info(
                    f"pickup/place complete; gripper current={average:.1f}mA; "
                    "SECTION2_DRIVE pose held"
                )
                return

            if self.grasp_attempt_count >= cfg.MAX_GRASP_RETRY:
                self.grasp_final_failed = True
                self.get_logger().error(
                    f"grasp failed; gripper current={average:.1f}mA"
                )
                return

            self.driver.write_gripper_position(cfg.GRIPPER_OPEN_POSITION)
            self.driver.write_wrist_position(cfg.RETRY_WRIST_POSITION)
            self.ready_stable_count = 0
            self.ready_pose = None
            self.grasp_sequence_started = False
            self.auto_control = True
            self._publish_hold_ready(False)
            self.visual_servo.reset()
            self.get_logger().warning("grasp verification failed; tracking resumed")
        except Exception as error:
            self.grasp_final_failed = True
            self.get_logger().error(f"grasp sequence exception: {error}")

    def image_callback(self, msg):
        if not self.perception_enabled or self.perception is None:
            return
        now = time.monotonic()
        if now - self.last_inference_time < 1.0 / cfg.INFERENCE_RATE_HZ:
            return
        self.last_inference_time = now

        frame = self.perception.image_to_bgr(msg)
        if frame is None or not self._validate_target_geometry(frame):
            return

        detection = self.perception.detect_box(frame)
        image_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        depth_mm = self.perception.get_box_depth_mm(detection, image_stamp_ns)
        self._publish_observation(msg.header, frame, detection, depth_mm)
        servo_state = "AUTO OFF"

        if self.grasp_final_failed:
            servo_state = "GRASP FAILED"
        elif self.grasp_sequence_finished:
            servo_state = "GRASP COMPLETE"
        elif self.grasp_sequence_started:
            servo_state = "GRASP RUNNING"
        elif detection is not None:
            cx, cy = detection["cx"], detection["cy"]
            x_ok = abs(cx - cfg.TARGET_X) <= cfg.DEAD_ZONE_X
            y_ok = abs(cy - cfg.TARGET_Y) <= cfg.DEAD_ZONE_Y
            depth_ok = depth_mm is not None and depth_mm <= cfg.TARGET_DEPTH_MM

            if not (x_ok and y_ok and depth_ok):
                self.ready_stable_count = 0

            if not self.auto_control:
                servo_state = "OBSERVE"
            elif not x_ok:
                servo_state = self.visual_servo.command_x_axis(cx)
            elif not y_ok:
                servo_state = self.visual_servo.command_y_axis(cy)
            elif depth_mm is None:
                servo_state = "DEPTH WAIT"
            elif depth_ok:
                self.ready_stable_count += 1
                servo_state = (
                    f"READY {self.ready_stable_count}/{cfg.READY_STABLE_FRAMES}"
                )
                if self.ready_stable_count >= cfg.READY_STABLE_FRAMES:
                    servo_state = self._start_grasp()
            else:
                servo_state = self.visual_servo.command_z_approach()
        else:
            self.ready_stable_count = 0

        if cfg.VISUALIZE:
            self.perception.draw_overlay(
                frame,
                detection,
                depth_mm,
                servo_state,
                self.auto_control,
            )
            cv2.imshow(cfg.WINDOW_NAME, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("a"):
                request = not self.auto_control
                if request and not cfg.ENABLE_MOTION:
                    self.get_logger().warning("auto control blocked: motion disabled")
                else:
                    self.auto_control = request
                    self._publish_hold_ready(
                        bool(cfg.ENABLE_MOTION) and not self.auto_control
                    )
            elif key == ord("q"):
                rclpy.shutdown()

    def close(self):
        self.auto_control = False
        if rclpy.ok():
            self._publish_hold_ready(False)
        self.driver.close_ports()
        if cfg.ENABLE_MOTION and not cfg.TORQUE_OFF_ON_EXIT:
            print(
                "[ROKA-ARM][LATCHED] 터미널 종료 후에도 마지막 자세 토크 유지",
                flush=True,
            )
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ArmControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"[ROKA-ARM][ERROR] {error}", flush=True)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.close()
                node.destroy_node()
            except KeyboardInterrupt:
                # A launch-level SIGINT may arrive while rclpy is destroying
                # entities. The command has already been stopped safely.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return exit_code


if __name__ == "__main__":
    main()
