"""Save the 2D, 3D, pose-graph, and terrain products from one mapping run."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_srvs.srv import Trigger

from roka_nav2_bringup.optimized_map_products import (
    derive_sc_pgo_navigation_products,
)
from roka_robot_description.workspace import workspace_path


class BundleSaver(Node):
    def __init__(
        self,
        directory: Path,
        *,
        environment: str = "unspecified",
        mapping_backend: str = "slam_toolbox_2d",
        use_global_arguments=True,
    ):
        super().__init__(
            "save_map_bundle",
            use_global_arguments=use_global_arguments,
        )
        self.directory = directory
        self.environment = environment
        self.mapping_backend = mapping_backend.strip().lower()

    def call(self, service_type, name, request, required=True):
        client = self.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=3.0):
            if required:
                raise RuntimeError(f"required service is unavailable: {name}")
            self.get_logger().warning(f"optional service unavailable: {name}")
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"service failed or timed out: {name}")
        return future.result()

    def save(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        if self.mapping_backend not in {"slam_toolbox_2d", "fastlio_sc_pgo"}:
            raise RuntimeError(f"unsupported mapping backend: {self.mapping_backend}")

        if self.mapping_backend == "slam_toolbox_2d":
            save_2d = SaveMap.Request()
            save_2d.name.data = str(self.directory / "occupancy")
            map_result = self.call(SaveMap, "/slam_toolbox/save_map", save_2d)
            if map_result.result != SaveMap.Response.RESULT_SUCCESS:
                raise RuntimeError(f"2D map save returned code {map_result.result}")

            serialize = SerializePoseGraph.Request()
            serialize.filename = str(self.directory / "slam_posegraph")
            pose_result = self.call(
                SerializePoseGraph, "/slam_toolbox/serialize_map", serialize
            )
            if pose_result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
                raise RuntimeError(f"pose graph save returned code {pose_result.result}")

        terrain_result = self.call(
            Trigger, "/terrain_mapper/save_map", Trigger.Request()
        )
        if not terrain_result.success:
            raise RuntimeError(terrain_result.message)

        # The selected 3D backend is asynchronous.  Retry briefly while its
        # final keyframe/graph update completes.
        graph_service = (
            "/pgo/save_map" if self.mapping_backend == "fastlio_sc_pgo" else ""
        )
        if graph_service:
            graph_deadline = time.monotonic() + 10.0
            graph_detail = "3D pose graph is not ready"
            while time.monotonic() < graph_deadline:
                graph_result = self.call(
                    Trigger,
                    graph_service,
                    Trigger.Request(),
                )
                graph_detail = graph_result.message
                if graph_result.success:
                    break
                time.sleep(0.25)
            else:
                raise RuntimeError(graph_detail)

        if self.mapping_backend == "fastlio_sc_pgo":
            derive_sc_pgo_navigation_products(self.directory)

        track_profile = self.directory / "track_profile.yaml"
        if not track_profile.exists():
            track_profile.write_text(
                "schema_version: 1\n"
                "frame_id: map\n"
                "closed_loop: true\n"
                "autonomy_approved: false\n"
                "approval_reason: raw_mapping_path_requires_traversability_annotation\n"
                "track_width_m: 0.9144\n"
                "default_speed_mps: 0.20\n"
                "points: []\n",
                encoding="utf-8",
            )
        metadata = self.directory / "metadata.yaml"
        products = {
            "occupancy": "occupancy.yaml",
            "terrain": "terrain_map.bag",
            "track_profile": "track_profile.yaml",
            "sensor_recording": "sensors/metadata.yaml",
            "session_metrics": "session_metrics.yaml",
        }
        if self.mapping_backend == "slam_toolbox_2d":
            products["slam_posegraph"] = "slam_posegraph.posegraph"
        if self.mapping_backend == "fastlio_sc_pgo":
            products.update({
                "pointcloud": "map.pcd",
                "optimized_path": "optimized_path.csv",
                "keyframe_poses": "keyframe_poses.bin",
                "scan_context": "scan_context.bin",
                "pose_graph_3d": "pose_graph.g2o",
                "pgo_metrics": "pgo_metrics.yaml",
            })
        metadata_document = {
            "schema_version": 2,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "environment": self.environment,
            "mapping_backend": self.mapping_backend,
            "pose_authority": (
                "slam_toolbox" if self.mapping_backend == "slam_toolbox_2d"
                else "sc_pgo"
            ),
            "frames": {
                "global": "map", "odometry": "odom",
                "body_3d": "base_link", "body_2d": "base_footprint",
            },
            "sensors": {"lidar": "Velodyne VLP-16", "imu": "EBIMU-9DOFV5"},
            "products": products,
        }
        metadata.write_text(
            yaml.safe_dump(
                metadata_document, sort_keys=False, allow_unicode=True
            ),
            encoding="utf-8",
        )
        return terrain_result.message


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "directory",
        nargs="?",
        default=str(workspace_path("maps", "track_v1")),
    )
    args = parser.parse_args(argv)
    rclpy.init()
    node = BundleSaver(Path(args.directory).expanduser().resolve())
    try:
        result = node.save()
        print(f"map bundle saved: {node.directory}\n{result}")
    except Exception as error:
        print(f"map bundle save failed: {error}", file=sys.stderr)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
