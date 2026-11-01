"""Expose the existing map-bundle saver as a guarded ROS service."""

from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

from roka_fast_lio_localization.bag_sensor_audit import audit_bag
from roka_robot_description.workspace import workspace_path

from roka_nav2_bringup.map_bundle_quality import (
    promote_bundle,
    validate_acquisition_bundle,
    validate_bundle,
    write_manifest,
)
from roka_nav2_bringup.save_map_bundle import BundleSaver


class MapBundleServer(Node):
    def __init__(self):
        super().__init__("map_bundle_server")
        self.declare_parameter(
            "directory", str(workspace_path("maps", "track_v1"))
        )
        self.declare_parameter("final_directory", "")
        self.declare_parameter("overwrite_existing", False)
        self.declare_parameter("require_sensor_bag", True)
        self.declare_parameter("environment", "unspecified")
        self.declare_parameter("mapping_backend", "slam_toolbox_2d")
        self.declare_parameter("acquisition_only", False)
        self.directory = Path(
            str(self.get_parameter("directory").value)
        ).expanduser().resolve()
        configured_final = str(self.get_parameter("final_directory").value).strip()
        self.final_directory = (
            Path(configured_final).expanduser().resolve()
            if configured_final
            else self.directory
        )
        self.overwrite_existing = bool(
            self.get_parameter("overwrite_existing").value
        )
        self.require_sensor_bag = bool(
            self.get_parameter("require_sensor_bag").value
        )
        self.environment = str(self.get_parameter("environment").value).strip().lower()
        self.mapping_backend = str(
            self.get_parameter("mapping_backend").value
        ).strip().lower()
        self.acquisition_only = bool(
            self.get_parameter("acquisition_only").value
        )
        self.busy = False
        self.create_service(Trigger, "~/save", self.save_callback)

    def save_callback(self, _request, response):
        if self.busy:
            response.success = False
            response.message = "map bundle save is already running"
            return response
        self.busy = True
        # The process-wide launch arguments remap every node to
        # ``map_bundle_server``.  The temporary service client must ignore that
        # remap or rclcpp/rclpy registers two rosout publishers with one name.
        helper = None
        try:
            if not self.acquisition_only:
                helper = BundleSaver(
                    self.directory,
                    environment=self.environment,
                    mapping_backend=self.mapping_backend,
                    use_global_arguments=False,
                )
                helper.save()
            if self.require_sensor_bag:
                sensor_contract = audit_bag(
                    self.directory / "sensors", require_diagnostics=True
                )
                contract_path = self.directory / "sensor_contract.yaml"
                temporary_contract = self.directory / ".sensor_contract.yaml.preparing"
                temporary_contract.write_text(
                    yaml.safe_dump(
                        sensor_contract, sort_keys=False, allow_unicode=True
                    ),
                    encoding="utf-8",
                )
                temporary_contract.replace(contract_path)
                if sensor_contract.get("result") != "PASS":
                    raise ValueError(
                        "sensor bag failed strict audit: "
                        + ", ".join(sensor_contract.get("failures") or [])
                    )
            if self.acquisition_only:
                metadata = {
                    "schema_version": 2,
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "environment": "capture",
                    "mapping_backend": "raw_sensor_capture",
                    "pose_authority": "wheel_imu_odom",
                    "frames": {
                        "global": "odom", "odometry": "odom",
                        "body_3d": "base_link", "body_2d": "base_footprint",
                    },
                    "products": {
                        "sensor_recording": "sensors/metadata.yaml",
                        "recorded_route": "track_profile.yaml",
                        "session_metrics": "session_metrics.yaml",
                    },
                }
                (self.directory / "metadata.yaml").write_text(
                    yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True),
                    encoding="utf-8",
                )
                metrics = validate_acquisition_bundle(self.directory)
            else:
                metrics = validate_bundle(
                    self.directory,
                    require_sensor_bag=self.require_sensor_bag,
                )
            write_manifest(self.directory, metrics)
            backup = promote_bundle(
                self.directory,
                self.final_directory,
                overwrite_existing=self.overwrite_existing,
            )
            response.success = True
            backup_detail = f"; previous={backup}" if backup is not None else ""
            terrain_message = (
                "raw acquisition saved; run the 2D/3D processor next"
                if self.acquisition_only
                else f"saved terrain map to {self.final_directory / 'terrain_map.bag'}"
            )
            response.message = (
                f"validated and committed map bundle to {self.final_directory}"
                f"{backup_detail}; {terrain_message}"
            )
        except Exception as error:  # report every partial-save/service failure
            response.success = False
            response.message = str(error)
        finally:
            if helper is not None:
                helper.destroy_node()
            self.busy = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapBundleServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
