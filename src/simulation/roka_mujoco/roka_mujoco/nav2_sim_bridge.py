"""Bridge MuJoCo ground truth into the real Nav2/safety interfaces for tests."""

from __future__ import annotations

from copy import deepcopy
import json
import math
import struct
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster
import yaml

from roka_nav2_bringup.route_geometry import prepare_centerline, project_pose


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def roll_pitch_from_quaternion(quaternion) -> tuple[float, float]:
    sin_roll = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z
    )
    cos_roll = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y
    )
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x
    )
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


class Nav2SimBridge(Node):
    """Expose wall-time, real-stack-shaped topics from the MuJoCo vehicle.

    Localization and the flat point cloud are explicit test oracles. This node
    validates Nav2, the speed/slope/slip supervisor, collision monitor and the
    final vehicle dynamics together; it does not claim to validate real AMCL
    or VLP-16 perception.
    """

    def __init__(self) -> None:
        super().__init__("nav2_sim_bridge")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("ground_truth_topic", "/simulator/floating_base_state")
        self.declare_parameter("sim_imu_topic", "/sim_imu_broadcaster/imu")
        self.declare_parameter("command_input_topic", "/cmd_vel")
        self.declare_parameter(
            "command_output_topic", "/sim_diff_drive_controller/cmd_vel_unstamped"
        )
        self.declare_parameter("cloud_publish_rate_hz", 5.0)
        self.declare_parameter("status_publish_rate_hz", 20.0)
        # Match the continuous follower/Nav2 open-section endpoint region.
        self.declare_parameter("goal_tolerance_m", 0.30)
        self.declare_parameter("maximum_lateral_error_m", 0.45)
        self.declare_parameter("align_profile_start_to_sim_origin", True)

        profile_path = Path(
            str(self.get_parameter("profile_file").value)
        ).expanduser()
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
        source_points = [dict(point) for point in document.get("points", [])]
        if len(source_points) < 2:
            raise ValueError("Nav2 simulation bridge needs a non-empty profile")
        self.closed_loop = bool(document.get("closed_loop", False))
        self.points, _ = prepare_centerline(
            source_points, closed=self.closed_loop, spacing_m=0.05,
            maximum_deviation_m=0.08,
        )
        self.align_profile_start = bool(
            self.get_parameter("align_profile_start_to_sim_origin").value
        )
        self.profile_origin_x = float(self.points[0]["x"]) if self.align_profile_start else 0.0
        self.profile_origin_y = float(self.points[0]["y"]) if self.align_profile_start else 0.0
        self.profile_origin_yaw = float(self.points[0].get("yaw", 0.0)) if self.align_profile_start else 0.0
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.maximum_lateral_error = float(
            self.get_parameter("maximum_lateral_error_m").value
        )

        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/amcl_pose", 10
        )
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.imu_connected_pub = self.create_publisher(Bool, "/imu/connected", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/velodyne_points", qos_profile_sensor_data
        )
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_stabilized", qos_profile_sensor_data
        )
        self.command_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("command_output_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(String, "/sim/nav2/status", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ground_truth_topic").value),
            self._ground_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("sim_imu_topic").value),
            self._imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("command_input_topic").value),
            self._command,
            10,
        )
        cloud_rate = float(self.get_parameter("cloud_publish_rate_hz").value)
        status_rate = float(self.get_parameter("status_publish_rate_hz").value)
        if min(cloud_rate, status_rate) <= 0.0:
            raise ValueError("bridge publication rates must be positive")
        self.create_timer(1.0 / cloud_rate, self._publish_flat_sensors)
        self.create_timer(1.0 / status_rate, self._publish_status)
        self.latest = None
        self.latest_roll = 0.0
        self.latest_pitch = 0.0
        self.command = Twist()
        self.started_at = self.get_clock().now()
        self.completed = False
        self.fault_reason = ""
        self.maximum_progress = 0.0
        self.maximum_lateral = 0.0
        self.command_started = False
        self.route_progress_index = None

    def _ground_truth(self, source: Odometry) -> None:
        now = self.get_clock().now().to_msg()
        message = deepcopy(source)
        message.header.stamp = now
        message.header.frame_id = "odom"
        message.child_frame_id = "base_footprint"
        source_x = float(source.pose.pose.position.x)
        source_y = float(source.pose.pose.position.y)
        cosine = math.cos(self.profile_origin_yaw)
        sine = math.sin(self.profile_origin_yaw)
        message.pose.pose.position.x = (
            self.profile_origin_x + cosine * source_x - sine * source_y
        )
        message.pose.pose.position.y = (
            self.profile_origin_y + sine * source_x + cosine * source_y
        )
        message.pose.pose.position.z = 0.0
        source_yaw = yaw_from_quaternion(source.pose.pose.orientation)
        yaw = self.profile_origin_yaw + source_yaw
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        source_world_x = float(source.twist.twist.linear.x)
        source_world_y = float(source.twist.twist.linear.y)
        world_x = cosine * source_world_x - sine * source_world_y
        world_y = sine * source_world_x + cosine * source_world_y
        message.twist.twist.linear.x = math.cos(yaw) * world_x + math.sin(yaw) * world_y
        message.twist.twist.linear.y = -math.sin(yaw) * world_x + math.cos(yaw) * world_y
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = math.radians(1.0) ** 2
        message.twist.covariance[0] = 0.0025
        message.twist.covariance[35] = 0.0025
        self.odom_pub.publish(message)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose = deepcopy(message.pose)
        self.pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.latest_roll, self.latest_pitch = roll_pitch_from_quaternion(
            source.pose.pose.orientation
        )
        self.latest = message
        self._update_episode_state()

    def _imu(self, source: Imu) -> None:
        message = deepcopy(source)
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "imu_link"
        # The MuJoCo keyframe uses local yaw zero while the selected mapped
        # route may begin at any map heading. Apply the same map-origin yaw to
        # the simulated world-referenced IMU orientation.
        half_yaw = 0.5 * self.profile_origin_yaw
        c, s = math.cos(half_yaw), math.sin(half_yaw)
        q = source.orientation
        message.orientation.x = c * q.x - s * q.y
        message.orientation.y = c * q.y + s * q.x
        message.orientation.z = c * q.z + s * q.w
        message.orientation.w = c * q.w - s * q.z
        message.orientation_covariance[0] = 0.035 ** 2
        message.orientation_covariance[4] = 0.035 ** 2
        message.orientation_covariance[8] = 0.175 ** 2
        message.angular_velocity_covariance[0] = 0.010344 ** 2
        message.angular_velocity_covariance[4] = 0.010344 ** 2
        message.angular_velocity_covariance[8] = 0.010344 ** 2
        message.linear_acceleration_covariance[0] = 0.02 ** 2
        message.linear_acceleration_covariance[4] = 0.02 ** 2
        message.linear_acceleration_covariance[8] = 0.02 ** 2
        self.imu_pub.publish(message)
        self.imu_connected_pub.publish(Bool(data=True))

    def _command(self, message: Twist) -> None:
        self.command = deepcopy(message)
        if (
            not self.command_started
            and not self.completed
            and math.hypot(message.linear.x, message.angular.z) > 0.005
        ):
            self.command_started = True
            self.started_at = self.get_clock().now()
        self.command_pub.publish(message)

    def _nearest(self) -> tuple[int, float]:
        if self.latest is None:
            return 0, math.inf
        x = self.latest.pose.pose.position.x
        y = self.latest.pose.pose.position.y
        yaw = yaw_from_quaternion(self.latest.pose.pose.orientation)
        projection = project_pose(
            self.points,
            x,
            y,
            yaw,
            closed=self.closed_loop,
            previous_unwrapped_index=self.route_progress_index,
            backward_window=4,
            forward_window=40,
        )
        if self.route_progress_index is None:
            self.route_progress_index = projection.unwrapped_index
        else:
            self.route_progress_index = max(
                self.route_progress_index, projection.unwrapped_index
            )
        index = min(len(self.points) - 1, self.route_progress_index)
        return index, projection.distance_m

    def _update_episode_state(self) -> None:
        index, lateral = self._nearest()
        progress = 100.0 * index / max(1, len(self.points) - 1)
        self.maximum_progress = max(self.maximum_progress, progress)
        self.maximum_lateral = max(self.maximum_lateral, lateral)
        if lateral > self.maximum_lateral_error and index > 1:
            self.fault_reason = "Nav2 vehicle left the mapped path corridor"
        final = self.points[-1]
        x = self.latest.pose.pose.position.x
        y = self.latest.pose.pose.position.y
        goal_distance = math.hypot(float(final["x"]) - x, float(final["y"]) - y)
        command_speed = math.hypot(
            self.command.linear.x, self.command.angular.z
        )
        if goal_distance <= self.goal_tolerance and command_speed < 0.02:
            self.completed = True
        start = self.points[0]
        start_distance = math.hypot(float(start["x"]) - x, float(start["y"]) - y)
        if self.completed and start_distance < 0.20 and goal_distance > 0.50:
            self.completed = False
            self.fault_reason = ""
            self.maximum_progress = 0.0
            self.maximum_lateral = 0.0
            self.started_at = self.get_clock().now()
            self.command_started = False
            self.route_progress_index = None

    @staticmethod
    def _flat_cloud(stamp, center_x: float, center_y: float) -> PointCloud2:
        values = []
        for ix in range(-20, 21):
            for iy in range(-20, 21):
                values.append((center_x + 0.10 * ix, center_y + 0.10 * iy, 0.0))
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.height = 1
        message.width = len(values)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(values)
        message.is_dense = True
        message.data = b"".join(struct.pack("<fff", *value) for value in values)
        return message

    def _publish_flat_sensors(self) -> None:
        if self.latest is None:
            return
        # Match the most recent odom/TF sample exactly. Stamping the cloud with
        # a newer timer time asks CollisionMonitor to extrapolate TF forward.
        stamp = self.latest.header.stamp
        x = self.latest.pose.pose.position.x
        y = self.latest.pose.pose.position.y
        self.cloud_pub.publish(self._flat_cloud(stamp, x, y))
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.20
        scan.range_min = 0.30
        scan.range_max = 30.0
        scan.ranges = [math.inf] * 361
        self.scan_pub.publish(scan)

    def _publish_status(self) -> None:
        if self.latest is None:
            return
        index, lateral = self._nearest()
        yaw = yaw_from_quaternion(self.latest.pose.pose.orientation)
        reference_yaw = float(self.points[index].get("yaw", 0.0))
        heading = math.degrees(math.atan2(
            math.sin(reference_yaw - yaw), math.cos(reference_yaw - yaw)
        ))
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds * 1.0e-9
        active = self.command_started and not self.completed and not self.fault_reason
        status = {
            "active": active,
            "completed_laps": 1 if self.completed else 0,
            "fault_reason": self.fault_reason,
            "episode_elapsed_sim_sec": elapsed,
            "path_progress_pct": 100.0 * index / max(1, len(self.points) - 1),
            "reference_lateral_error_m": lateral,
            "reference_heading_error_deg": heading,
            "roll_deg": math.degrees(self.latest_roll),
            "pitch_deg": math.degrees(self.latest_pitch),
            "corridor_available": lateral <= self.maximum_lateral_error,
            "command_linear_mps": self.command.linear.x,
            "command_angular_radps": self.command.angular.z,
            "recovery_active": False,
            "recovery_count": 0,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # During launch SIGINT, Humble can invalidate the context between a
        # subscription take and Python message conversion.  Suppress only
        # that shutdown race; a runtime failure while ROS is active remains
        # a real error and must still reach the terminal.
        if "Unable to convert call argument to Python object" not in str(error):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
