"""Load and validate the single source of truth for ROKA hardware geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
import yaml


def _rpy_matrix(rpy: Sequence[float]) -> tuple[float, ...]:
    """ROS fixed-axis RPY rotation (Rz(yaw) * Ry(pitch) * Rx(roll))."""
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    )


def _transpose(matrix: Sequence[float]) -> tuple[float, ...]:
    return tuple(matrix[3 * column + row] for row in range(3) for column in range(3))


def _matrix_multiply(left: Sequence[float], right: Sequence[float]) -> tuple[float, ...]:
    return tuple(
        sum(left[3 * row + k] * right[3 * k + column] for k in range(3))
        for row in range(3)
        for column in range(3)
    )


def _matrix_vector(matrix: Sequence[float], vector: Sequence[float]) -> tuple[float, ...]:
    return tuple(
        sum(matrix[3 * row + column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _vector(document: Mapping[str, Any], key: str) -> tuple[float, float, float]:
    value = document.get(key)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{key} must contain exactly three numeric values")
    result = tuple(float(item) for item in value)
    if not all(abs(item) < 100.0 for item in result):
        raise ValueError(f"{key} contains an implausible value: {result}")
    return result


@dataclass(frozen=True)
class Transform:
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


@dataclass(frozen=True)
class HardwareConfig:
    path: Path
    coordinate_convention: str
    body_3d_frame: str
    body_2d_frame: str
    lidar_frame: str
    imu_frame: str
    base_to_lidar: Transform
    base_to_imu: Transform
    footprint_to_base: Transform
    wheel_radius_m: float
    wheelbase_m: float
    physical_wheel_track_m: float
    tire_half_width_m: float
    effective_drive_separation_m: float
    effective_drive_separation_status: str
    navigation_front_m: float
    navigation_rear_m: float
    navigation_half_width_m: float
    lidar_rate_hz: float
    imu_rate_hz: float
    time_offset_lidar_to_imu_sec: float
    rotation_validated_dynamically: bool
    temporal_offset_validated_dynamically: bool

    @property
    def wheel_separation_m(self) -> float:
        """Backward-compatible name for the effective skid-steer separation.

        Motor kinematics need the calibrated/effective separation.  Geometry,
        terrain and clearance code must use ``physical_wheel_track_m``.
        """
        return self.effective_drive_separation_m

    @property
    def navigation_length_m(self) -> float:
        return self.navigation_front_m + self.navigation_rear_m

    @property
    def navigation_width_m(self) -> float:
        return 2.0 * self.navigation_half_width_m

    @property
    def wheel_outer_width_m(self) -> float:
        return self.physical_wheel_track_m + 2.0 * self.tire_half_width_m

    @property
    def navigation_footprint(self) -> tuple[tuple[float, float], ...]:
        return (
            (self.navigation_front_m, self.navigation_half_width_m),
            (self.navigation_front_m, -self.navigation_half_width_m),
            (-self.navigation_rear_m, -self.navigation_half_width_m),
            (-self.navigation_rear_m, self.navigation_half_width_m),
        )

    @property
    def lidar_in_imu_translation(self) -> tuple[float, float, float]:
        """LiDAR origin relative to IMU, expressed in calibrated IMU axes."""
        base_difference = tuple(
            lidar - imu
            for lidar, imu in zip(self.base_to_lidar.xyz, self.base_to_imu.xyz)
        )
        return _matrix_vector(_transpose(self.base_to_imu_rotation_matrix), base_difference)

    @property
    def base_to_imu_rotation_matrix(self) -> tuple[float, ...]:
        """IMU axes expressed in base axes (R_base_imu)."""
        return _rpy_matrix(self.base_to_imu.rpy)

    @property
    def lidar_in_imu_rotation_matrix(self) -> tuple[float, ...]:
        base_to_lidar = _rpy_matrix(self.base_to_lidar.rpy)
        return _matrix_multiply(
            _transpose(self.base_to_imu_rotation_matrix), base_to_lidar
        )


def default_hardware_config_path() -> Path:
    return Path(get_package_share_directory("roka_robot_description")) / "config" / "pk3a_hardware.yaml"


def load_hardware_config(path: str | Path | None = None) -> HardwareConfig:
    config_path = Path(path).expanduser().resolve() if path else default_hardware_config_path()
    document = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if document.get("schema_version") != 2:
        raise ValueError(f"unsupported hardware schema in {config_path}")
    if document.get("coordinate_convention") != "ros_flu":
        raise ValueError("ROKA ROS processing requires the ros_flu convention")

    frames = document.get("frames") or {}
    geometry = document.get("geometry") or {}
    sensors = document.get("sensors") or {}
    calibration = (document.get("calibration") or {}).get("lidar_to_imu") or {}
    vehicle = document.get("vehicle") or {}
    footprint = vehicle.get("navigation_footprint") or {}

    def transform(name: str) -> Transform:
        entry = geometry.get(name) or {}
        return Transform(_vector(entry, "xyz_m"), _vector(entry, "rpy_rad"))

    result = HardwareConfig(
        path=config_path,
        coordinate_convention="ros_flu",
        body_3d_frame=str(frames.get("body_3d", "")),
        body_2d_frame=str(frames.get("body_2d", "")),
        lidar_frame=str(frames.get("lidar", "")),
        imu_frame=str(frames.get("imu", "")),
        base_to_lidar=transform("base_link_to_lidar"),
        base_to_imu=transform("base_link_to_imu"),
        footprint_to_base=transform("base_footprint_to_base_link"),
        wheel_radius_m=float(vehicle.get("wheel_radius_m")),
        wheelbase_m=float(vehicle.get("wheelbase_m")),
        physical_wheel_track_m=float(vehicle.get("physical_wheel_track_m")),
        tire_half_width_m=float(vehicle.get("tire_half_width_m")),
        effective_drive_separation_m=float(
            vehicle.get("effective_drive_separation_m")
        ),
        effective_drive_separation_status=str(
            vehicle.get("effective_drive_separation_status", "unknown")
        ),
        navigation_front_m=float(footprint.get("front_m")),
        navigation_rear_m=float(footprint.get("rear_m")),
        navigation_half_width_m=float(footprint.get("half_width_m")),
        lidar_rate_hz=float((sensors.get("lidar") or {}).get("scan_rate_hz")),
        imu_rate_hz=float((sensors.get("imu") or {}).get("sample_rate_hz")),
        time_offset_lidar_to_imu_sec=float(calibration.get("time_offset_lidar_to_imu_sec", 0.0)),
        rotation_validated_dynamically=bool(calibration.get("rotation_validated_dynamically", False)),
        temporal_offset_validated_dynamically=bool(
            calibration.get("temporal_offset_validated_dynamically", False)
        ),
    )
    if not all((result.body_3d_frame, result.body_2d_frame, result.lidar_frame, result.imu_frame)):
        raise ValueError("all canonical frame names must be non-empty")
    positive_dimensions = {
        "wheel_radius_m": result.wheel_radius_m,
        "wheelbase_m": result.wheelbase_m,
        "physical_wheel_track_m": result.physical_wheel_track_m,
        "tire_half_width_m": result.tire_half_width_m,
        "effective_drive_separation_m": result.effective_drive_separation_m,
        "navigation_front_m": result.navigation_front_m,
        "navigation_rear_m": result.navigation_rear_m,
        "navigation_half_width_m": result.navigation_half_width_m,
    }
    for name, value in positive_dimensions.items():
        if not math.isfinite(value) or not 0.01 < value < 2.0:
            raise ValueError(f"{name} is implausible")
    if result.wheel_outer_width_m > result.navigation_width_m:
        raise ValueError("navigation footprint must contain both tires")
    if result.navigation_length_m < result.wheelbase_m:
        raise ValueError("navigation footprint must contain both wheel axles")
    if not 1.0 <= result.lidar_rate_hz <= 100.0:
        raise ValueError("LiDAR rate is implausible")
    if not result.lidar_rate_hz < result.imu_rate_hz <= 1000.0:
        raise ValueError("IMU rate must be higher than LiDAR scan rate")
    # A VLP-16 revolution is delivered over roughly 0.10 s and this stack's
    # header is tied to the first packet.  Two-run LI-Init therefore may
    # legitimately select the adjacent 30 Hz cut-frame branch just beyond one
    # revolution.  Keep the same hard two-scan bound as the calibration
    # parser; provenance/repeatability checks below still gate its use.
    if not math.isfinite(result.time_offset_lidar_to_imu_sec) or abs(
        result.time_offset_lidar_to_imu_sec
    ) > 0.20:
        raise ValueError("LiDAR-IMU time offset is non-finite or implausible")
    if (
        result.rotation_validated_dynamically
        != result.temporal_offset_validated_dynamically
    ):
        raise ValueError(
            "LiDAR-IMU rotation and temporal dynamic validation must be approved together"
        )
    if result.rotation_validated_dynamically:
        expected_rotation_source = "li_init_dynamic_two_run_validation"
        accepted_temporal_sources = {
            expected_rotation_source,
            "li_init_dynamic_two_run_vlp16_cut_frame_phase_residual",
            "li_init_dynamic_two_run_vlp16_common_cut_frame_branch",
        }
        if (
            calibration.get("rotation_source") != expected_rotation_source
            or calibration.get("temporal_source") not in accepted_temporal_sources
        ):
            raise ValueError("dynamic calibration flags lack LI-Init two-run provenance")
        if (
            calibration.get("temporal_source")
            == "li_init_dynamic_two_run_vlp16_cut_frame_phase_residual"
            and calibration.get("time_offset_policy")
            != "preserve_raw_li_init_remove_whole_30hz_cut_frames"
        ):
            raise ValueError("VLP-16 phase-normalized timing lacks its policy evidence")
        if (
            calibration.get("temporal_source")
            == "li_init_dynamic_two_run_vlp16_common_cut_frame_branch"
            and calibration.get("time_offset_policy")
            != "preserve_raw_li_init_align_common_30hz_cut_frame_branch"
        ):
            raise ValueError("VLP-16 common-branch timing lacks its policy evidence")
        repeatability = calibration.get("validation_repeatability") or {}
        required_repeatability = (
            "rotation_difference_deg",
            "time_offset_difference_sec",
            "translation_difference_m",
        )
        if not calibration.get("validated_at_utc") or not all(
            key in repeatability for key in required_repeatability
        ):
            raise ValueError("dynamic calibration flags lack repeatability evidence")
        measured = tuple(
            float(repeatability[key]) for key in required_repeatability
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in measured):
            raise ValueError("dynamic calibration repeatability is invalid")
        # Translation remains the CAD/tape-measured rigid displacement.  A
        # wheeled trajectory does not observe LI-Init translation strongly
        # enough for its run-to-run delta to gate the rotation/time result.
        if measured[0] > 2.0 or measured[1] > 0.005:
            raise ValueError("dynamic calibration repeatability exceeds acceptance limits")
    result.lidar_in_imu_rotation_matrix
    return result
