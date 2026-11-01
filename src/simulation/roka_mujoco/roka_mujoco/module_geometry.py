"""Independent surface definitions and connector contracts for PK-3A modules."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple

import numpy as np


@dataclass(frozen=True)
class ModuleSpec:
    name: str
    page: int
    shape: str
    length: float
    width: float
    inlet: np.ndarray
    outlet: np.ndarray
    properties: Dict[str, Any]


def package_root() -> Path:
    source_root = Path(__file__).resolve().parents[1]
    if (source_root / "config" / "pk3a_modules.json").exists():
        return source_root
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("roka_mujoco"))
    except (ImportError, LookupError):
        return source_root


def default_modules_path() -> Path:
    return package_root() / "config" / "pk3a_modules.json"
def load_json(path: Path | str) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_module_document(path: Path | str | None = None) -> Dict[str, Any]:
    return load_json(path or default_modules_path())


def load_modules(path: Path | str | None = None) -> Dict[str, ModuleSpec]:
    document = load_module_document(path)
    result = {}
    for name, raw in document["modules"].items():
        result[name] = ModuleSpec(
            name=name,
            page=int(raw["page"]),
            shape=str(raw["shape"]),
            length=float(raw["length_m"]),
            width=float(raw["width_m"]),
            inlet=np.asarray(raw["inlet_right_left_m"], dtype=float),
            outlet=np.asarray(raw["outlet_right_left_m"], dtype=float),
            properties=dict(raw),
        )
    return result


def smoothstep(value: np.ndarray) -> np.ndarray:
    value = np.clip(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def _anchored_c1_fraction(value: np.ndarray, knot: float) -> np.ndarray:
    """Ease from zero to one while preserving an internal block boundary.

    The two Hermite pieces have zero derivatives at the outside ports and a
    unit derivative at ``knot``.  Therefore the physical 60/40 TB1L split is
    not moved when the course direction is softened.
    """
    if not 0.0 < knot < 1.0:
        raise ValueError(f"C1 knot must lie inside (0, 1), got {knot}")
    value = np.clip(np.asarray(value, dtype=float), 0.0, 1.0)

    left_parameter = np.clip(value / knot, 0.0, 1.0)
    left_h01 = -2.0 * left_parameter**3 + 3.0 * left_parameter**2
    left_h11 = left_parameter**3 - left_parameter**2
    left = knot * (left_h01 + left_h11)

    right_span = 1.0 - knot
    right_parameter = np.clip((value - knot) / right_span, 0.0, 1.0)
    right_h00 = 2.0 * right_parameter**3 - 3.0 * right_parameter**2 + 1.0
    right_h10 = (
        right_parameter**3
        - 2.0 * right_parameter**2
        + right_parameter
    )
    right_h01 = -2.0 * right_parameter**3 + 3.0 * right_parameter**2
    right = (
        right_h00 * knot
        + right_h10 * right_span
        + right_h01
    )
    return np.where(value <= knot, left, right)


def _transverse(profile: np.ndarray, lateral: np.ndarray) -> np.ndarray:
    """Interpolate a connector profile from right (-1) to left (+1)."""
    left_fraction = 0.5 * (np.clip(lateral, -1.0, 1.0) + 1.0)
    return profile[0] * (1.0 - left_fraction) + profile[1] * left_fraction


def _cubic_bezier_profile(
    progress: np.ndarray,
    length: float,
    start_height: float,
    end_height: float,
    start_slope_deg: float,
    end_slope_deg: float,
    start_handle_fraction: float,
    end_handle_fraction: float,
) -> np.ndarray:
    """Evaluate a drawing-derived profile with explicit end tangent angles.

    The engineering sheets give heights and tangent angles but not the curve's
    centre/radius.  A parametric cubic Bezier preserves those stated boundary
    conditions without pretending that an undocumented circular radius exists.
    """
    start_dx = length * start_handle_fraction
    end_dx = length * end_handle_fraction
    control_x = np.array([0.0, start_dx, length - end_dx, length])
    control_z = np.array([
        start_height,
        start_height + np.tan(np.deg2rad(start_slope_deg)) * start_dx,
        end_height - np.tan(np.deg2rad(end_slope_deg)) * end_dx,
        end_height,
    ])
    parameter = np.linspace(0.0, 1.0, 2049)
    inverse = 1.0 - parameter
    weights = np.column_stack([
        inverse**3,
        3.0 * inverse**2 * parameter,
        3.0 * inverse * parameter**2,
        parameter**3,
    ])
    curve_x = weights @ control_x
    curve_z = weights @ control_z
    return np.interp(np.clip(progress, 0.0, length), curve_x, curve_z)


def _line_spline_line_profile(
    progress: np.ndarray,
    length: float,
    start_height: float,
    end_height: float,
    start_slope_deg: float,
    end_slope_deg: float,
    start_line_fraction: float,
    end_line_fraction: float,
) -> np.ndarray:
    """CNC-style cutter path: tangent line, cubic Hermite, tangent line."""
    progress = np.clip(np.asarray(progress, dtype=float), 0.0, length)
    start_x = start_line_fraction * length
    end_x = (1.0 - end_line_fraction) * length
    start_slope = np.tan(np.deg2rad(start_slope_deg))
    end_slope = np.tan(np.deg2rad(end_slope_deg))
    start_z = start_height + start_slope * start_x
    end_z = end_height - end_slope * (length - end_x)
    span = end_x - start_x
    parameter = np.clip((progress - start_x) / span, 0.0, 1.0)
    h00 = 2.0 * parameter**3 - 3.0 * parameter**2 + 1.0
    h10 = parameter**3 - 2.0 * parameter**2 + parameter
    h01 = -2.0 * parameter**3 + 3.0 * parameter**2
    h11 = parameter**3 - parameter**2
    spline = (
        h00 * start_z
        + h10 * span * start_slope
        + h01 * end_z
        + h11 * span * end_slope
    )
    start_line = start_height + start_slope * progress
    end_line = end_height - end_slope * (length - progress)
    return np.where(
        progress < start_x,
        start_line,
        np.where(progress > end_x, end_line, spline),
    )


def tb1l_compound_profiles(
    spec: ModuleSpec, progress: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return right outer, shared seam, and left outer TB1L splines."""
    progress = np.asarray(progress, dtype=float)
    if spec.properties.get("profile_mode") == "line_spline_line_cnc":
        right = _line_spline_line_profile(
            progress,
            spec.length,
            float(spec.inlet[0]),
            float(spec.outlet[0]),
            float(spec.properties["right_start_slope_deg"]),
            float(spec.properties["right_end_slope_deg"]),
            float(spec.properties["right_start_line_fraction"]),
            float(spec.properties["right_end_line_fraction"]),
        )
        left = _line_spline_line_profile(
            progress,
            spec.length,
            float(spec.inlet[1]),
            float(spec.outlet[1]),
            float(spec.properties["left_start_slope_deg"]),
            float(spec.properties["left_end_slope_deg"]),
            float(spec.properties["left_start_line_fraction"]),
            float(spec.properties["left_end_line_fraction"]),
        )
    else:
        right = _cubic_bezier_profile(
            progress,
            spec.length,
            float(spec.inlet[0]),
            float(spec.outlet[0]),
            float(spec.properties["right_start_slope_deg"]),
            float(spec.properties["right_end_slope_deg"]),
            float(spec.properties["right_start_handle_fraction"]),
            float(spec.properties["right_end_handle_fraction"]),
        )
        left = _cubic_bezier_profile(
            progress,
            spec.length,
            float(spec.inlet[1]),
            float(spec.outlet[1]),
            float(spec.properties["left_start_slope_deg"]),
            float(spec.properties["left_end_slope_deg"]),
            float(spec.properties["left_start_handle_fraction"]),
            float(spec.properties["left_end_handle_fraction"]),
        )
    seam_weight = float(spec.properties["seam_left_weight"])
    seam = right * (1.0 - seam_weight) + left * seam_weight
    return right, seam, left


def surface_height(spec: ModuleSpec, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Evaluate a module in its local inlet-to-outlet coordinate system."""
    mirror_source = spec.properties.get("_mirror_source_spec")
    if mirror_source is not None:
        return surface_height(
            mirror_source,
            np.asarray(x, dtype=float),
            -np.asarray(y, dtype=float),
        )
    course_axis_source = spec.properties.get("_course_axis_source_spec")
    if course_axis_source is not None:
        course_fraction = np.clip(
            np.asarray(x, dtype=float) / spec.length, 0.0, 1.0
        )
        blend_mode = spec.properties.get("course_axis_blend_mode", "linear")
        if blend_mode == "linear":
            source_fraction = course_fraction
        elif blend_mode == "c1_smoothstep":
            source_fraction = smoothstep(course_fraction)
        elif blend_mode == "c1_two_block":
            source_fraction = _anchored_c1_fraction(
                course_fraction,
                float(spec.properties["course_axis_knot_fraction"]),
            )
        else:
            raise ValueError(f"Unsupported course-axis blend: {blend_mode}")
        return surface_height(
            course_axis_source,
            np.asarray(y, dtype=float) + 0.5 * course_axis_source.length,
            source_fraction * course_axis_source.width
            - 0.5 * course_axis_source.width,
        )
    u = np.clip(np.asarray(x, dtype=float) / spec.length, 0.0, 1.0)
    lateral = np.clip(2.0 * np.asarray(y, dtype=float) / spec.width, -1.0, 1.0)
    inlet = _transverse(spec.inlet, lateral)
    outlet = _transverse(spec.outlet, lateral)

    if spec.shape == "half_roller":
        blend = 0.5 * (1.0 - np.cos(np.pi * u))
        return inlet * (1.0 - blend) + outlet * blend
    if spec.shape == "double_bowl":
        centre = float(spec.properties["centre_height_m"])
        return centre + (inlet - centre) * np.cos(np.pi * u) ** 2
    if spec.shape == "linear_joiner":
        return inlet * (1.0 - u) + outlet * u
    if spec.shape == "vertical_curve":
        slope_in = np.tan(np.deg2rad(float(spec.properties["inlet_slope_deg"])))
        slope_out = np.tan(np.deg2rad(float(spec.properties["outlet_slope_deg"])))
        h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
        h10 = u**3 - 2.0 * u**2 + u
        h01 = -2.0 * u**3 + 3.0 * u**2
        h11 = u**3 - u**2
        value = h00 * inlet + h10 * spec.length * slope_in + h01 * outlet + h11 * spec.length * slope_out
        return np.clip(value, np.minimum(inlet, outlet), np.maximum(inlet, outlet))
    if spec.shape == "constant_bank_left":
        if spec.properties.get("bank_profile_mode") == "line_spline_line_cnc":
            low_to_high_distance = 0.5 * spec.width - np.asarray(y, dtype=float)
            return _line_spline_line_profile(
                low_to_high_distance,
                spec.width,
                float(spec.properties["bank_low_height_m"]),
                float(spec.properties["bank_high_height_m"]),
                float(spec.properties["bank_low_slope_deg"]),
                float(spec.properties["bank_high_slope_deg"]),
                float(spec.properties["bank_low_line_fraction"]),
                float(spec.properties["bank_high_line_fraction"]),
            )
        return inlet
    if spec.shape == "ta1l_drawing_profile":
        return _cubic_bezier_profile(
            u * spec.length,
            spec.length,
            float(spec.inlet[0]),
            float(spec.outlet[0]),
            float(spec.properties["start_slope_deg"]),
            float(spec.properties["end_slope_deg"]),
            float(spec.properties["start_handle_fraction"]),
            float(spec.properties["end_handle_fraction"]),
        )
    if spec.shape == "ta1l_asymmetric_spline_loft":
        right = np.full_like(u, float(spec.inlet[0]), dtype=float)
        if spec.properties.get("profile_mode") == "line_spline_line_cnc":
            left = _line_spline_line_profile(
                u * spec.length,
                spec.length,
                float(spec.inlet[1]),
                float(spec.outlet[1]),
                float(spec.properties["spline_start_slope_deg"]),
                float(spec.properties["spline_end_slope_deg"]),
                float(spec.properties["spline_start_line_fraction"]),
                float(spec.properties["spline_end_line_fraction"]),
            )
        else:
            left = _cubic_bezier_profile(
                u * spec.length,
                spec.length,
                float(spec.inlet[1]),
                float(spec.outlet[1]),
                float(spec.properties["spline_start_slope_deg"]),
                float(spec.properties["spline_end_slope_deg"]),
                float(spec.properties["spline_start_handle_fraction"]),
                float(spec.properties["spline_end_handle_fraction"]),
            )
        left_fraction = 0.5 * (lateral + 1.0)
        return right * (1.0 - left_fraction) + left * left_fraction
    if spec.shape == "tb1l_drawing_banked_transition":
        right = _cubic_bezier_profile(
            u * spec.length,
            spec.length,
            float(spec.inlet[0]),
            float(spec.outlet[0]),
            float(spec.properties["right_start_slope_deg"]),
            float(spec.properties["right_end_slope_deg"]),
            float(spec.properties["right_start_handle_fraction"]),
            float(spec.properties["right_end_handle_fraction"]),
        )
        left = _cubic_bezier_profile(
            u * spec.length,
            spec.length,
            float(spec.inlet[1]),
            float(spec.outlet[1]),
            float(spec.properties["left_start_slope_deg"]),
            float(spec.properties["left_end_slope_deg"]),
            float(spec.properties["left_start_handle_fraction"]),
            float(spec.properties["left_end_handle_fraction"]),
        )
        left_fraction = 0.5 * (lateral + 1.0)
        return right * (1.0 - left_fraction) + left * left_fraction
    if spec.shape == "tb1l_compound_two_block_spline":
        right, seam, left = tb1l_compound_profiles(spec, u * spec.length)
        seam_lateral = float(spec.properties.get("seam_lateral_normalized", 0.0))
        right_half = np.clip(
            (lateral + 1.0) / (seam_lateral + 1.0), 0.0, 1.0
        )
        left_half = np.clip(
            (lateral - seam_lateral) / (1.0 - seam_lateral), 0.0, 1.0
        )
        return np.where(
            lateral <= seam_lateral,
            right * (1.0 - right_half) + seam * right_half,
            seam * (1.0 - left_half) + left * left_half,
        )
    if spec.shape.startswith("bank_transition"):
        blend = smoothstep(u)
        return inlet * (1.0 - blend) + outlet * blend
    raise ValueError(f"Unsupported module shape: {spec.shape}")


def rasterize_module(
    spec: ModuleSpec, cell_size: float
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    mirror_source = spec.properties.get("_mirror_source_spec")
    if mirror_source is not None:
        height, coordinates = rasterize_module(mirror_source, cell_size)
        return height[::-1].copy(), {
            "x": coordinates["x"].copy(),
            "y": (-coordinates["y"][::-1]).copy(),
            "inside": coordinates["inside"][::-1].copy(),
        }
    bounds = module_bounds(spec)
    columns = max(3, int(round((bounds[1] - bounds[0]) / cell_size)) + 1)
    rows = max(3, int(round((bounds[3] - bounds[2]) / cell_size)) + 1)
    x = np.linspace(bounds[0], bounds[1], columns)
    y = np.linspace(bounds[3], bounds[2], rows)
    xx, yy = np.meshgrid(x, y)
    if not is_turn_module(spec):
        progress = xx + 0.5 * spec.length
        lateral = yy
        inside = np.ones_like(xx, dtype=bool)
    else:
        angle = np.deg2rad(float(spec.properties["turn_deg"]))
        half_angle = 0.5 * angle
        radius = spec.length / angle
        circle_y = radius * np.cos(half_angle)
        theta = np.arctan2(xx, circle_y - yy)
        radial = np.hypot(xx, yy - circle_y)
        lateral = radius - radial
        progress = (theta + half_angle) / angle * spec.length
        inside = (
            (theta >= -half_angle)
            & (theta <= half_angle)
            & (np.abs(lateral) <= 0.5 * spec.width)
        )
    height = np.zeros_like(xx)
    height[inside] = surface_height(spec, progress[inside], lateral[inside])
    return height, {"x": x, "y": y, "inside": inside}


def footprint_contains(
    spec: ModuleSpec,
    x: np.ndarray | float,
    y: np.ndarray | float,
    tolerance: float = 1.0e-9,
) -> np.ndarray:
    """Return whether local plan-view points lie on a module's riding plate."""
    x_values = np.asarray(x, dtype=float)
    y_values = np.asarray(y, dtype=float)
    mirror_source = spec.properties.get("_mirror_source_spec")
    if mirror_source is not None:
        return footprint_contains(
            mirror_source, x_values, -y_values, tolerance=tolerance
        )
    if not is_turn_module(spec):
        return (
            (np.abs(x_values) <= 0.5 * spec.length + tolerance)
            & (np.abs(y_values) <= 0.5 * spec.width + tolerance)
        )

    angle = np.deg2rad(float(spec.properties["turn_deg"]))
    half_angle = 0.5 * angle
    radius = spec.length / angle
    circle_y = radius * np.cos(half_angle)
    theta = np.arctan2(x_values, circle_y - y_values)
    radial = np.hypot(x_values, y_values - circle_y)
    lateral = radius - radial
    return (
        (theta >= -half_angle - tolerance)
        & (theta <= half_angle + tolerance)
        & (np.abs(lateral) <= 0.5 * spec.width + tolerance)
    )


def is_turn_module(spec: ModuleSpec) -> bool:
    return "turn_deg" in spec.properties


def connector_geometry(
    spec: ModuleSpec, reverse: bool = False
) -> Dict[str, np.ndarray | float]:
    """Local inlet/outlet points, headings, and hfield bounding-box centre."""
    bounds = module_bounds(spec)
    centre = np.array([0.5 * (bounds[0] + bounds[1]), 0.5 * (bounds[2] + bounds[3])])
    if not is_turn_module(spec):
        geometry = {
            "inlet_point": np.array([-0.5 * spec.length, 0.0]),
            "outlet_point": np.array([0.5 * spec.length, 0.0]),
            "inlet_heading": 0.0,
            "outlet_heading": 0.0,
            "bounds_centre": centre,
        }
    else:
        angle = np.deg2rad(float(spec.properties["turn_deg"]))
        radius = spec.length / angle
        half_angle = 0.5 * angle
        half_chord = radius * np.sin(half_angle)
        geometry = {
            "inlet_point": np.array([-half_chord, 0.0]),
            "outlet_point": np.array([half_chord, 0.0]),
            "inlet_heading": -half_angle,
            "outlet_heading": half_angle,
            "bounds_centre": centre,
        }
    if not reverse:
        return geometry
    return {
        "inlet_point": np.asarray(geometry["outlet_point"]),
        "outlet_point": np.asarray(geometry["inlet_point"]),
        "inlet_heading": float(geometry["outlet_heading"]) + np.pi,
        "outlet_heading": float(geometry["inlet_heading"]) + np.pi,
        "bounds_centre": centre,
    }


def surface_point(spec: ModuleSpec, progress: float, lateral: float) -> np.ndarray:
    """Map distance/lateral coordinates to a local XY point on the module."""
    mirror_source = spec.properties.get("_mirror_source_spec")
    if mirror_source is not None:
        point = surface_point(mirror_source, progress, -lateral).copy()
        point[1] *= -1.0
        return point
    if not is_turn_module(spec):
        return np.array([progress - 0.5 * spec.length, lateral])
    angle = np.deg2rad(float(spec.properties["turn_deg"]))
    radius = spec.length / angle
    half_angle = 0.5 * angle
    theta = progress / spec.length * angle - half_angle
    centreline = np.array([
        radius * np.sin(theta),
        radius * np.cos(half_angle) - radius * np.cos(theta),
    ])
    left_normal = np.array([-np.sin(theta), np.cos(theta)])
    return centreline + lateral * left_normal


def module_bounds(spec: ModuleSpec) -> np.ndarray:
    if not is_turn_module(spec):
        return np.array([
            -0.5 * spec.length,
            0.5 * spec.length,
            -0.5 * spec.width,
            0.5 * spec.width,
        ])
    samples = []
    for progress in np.linspace(0.0, spec.length, 101):
        samples.append(surface_point(spec, progress, -0.5 * spec.width))
        samples.append(surface_point(spec, progress, 0.5 * spec.width))
    points = np.asarray(samples)
    return np.array([
        float(points[:, 0].min()),
        float(points[:, 0].max()),
        float(points[:, 1].min()),
        float(points[:, 1].max()),
    ])


def connector_profiles(spec: ModuleSpec, reverse: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    if reverse:
        return spec.outlet[::-1].copy(), spec.inlet[::-1].copy()
    return spec.inlet.copy(), spec.outlet.copy()


def all_connector_values(modules: Iterable[ModuleSpec]) -> np.ndarray:
    return np.concatenate([np.concatenate([module.inlet, module.outlet]) for module in modules])


def mirror_module_y(spec: ModuleSpec, name: str | None = None) -> ModuleSpec:
    """Create a metric left/right reflection without scaling the source asset."""
    properties = dict(spec.properties)
    properties["_mirror_source_spec"] = spec
    properties["mirrored_y"] = True
    properties["mirror_source_name"] = spec.name
    if "turn_deg" in properties:
        properties["turn_deg"] = -float(properties["turn_deg"])
    return ModuleSpec(
        name=name or f"{spec.name}_MIRROR",
        page=spec.page,
        shape=f"mirror_y:{spec.shape}",
        length=spec.length,
        width=spec.width,
        inlet=spec.inlet[::-1].copy(),
        outlet=spec.outlet[::-1].copy(),
        properties=properties,
    )


def orient_course_across_width(
    spec: ModuleSpec,
    name: str | None = None,
    course_length: float | None = None,
    blend_mode: str = "linear",
) -> ModuleSpec:
    """Use the drawing's two longitudinal side profiles as course end ports.

    TA1L and TB1L are lofted between unequal side profiles.  Their approved
    drawing profiles were sampled along the original X axis, while the course
    progresses from the rectangular/low-angle side to the spline/high-angle
    side.  This adapter swaps the parametric axes.  An optional C1 blend can
    lengthen the physical course direction and reduce the inlet/outlet tangent
    to zero, preventing both a false 90-degree joint and a sharp rideable seam.
    """
    if is_turn_module(spec):
        raise ValueError("Course-axis reorientation is only valid for straight footprints")
    properties = dict(spec.properties)
    properties["_course_axis_source_spec"] = spec
    properties["course_axis_reoriented"] = True
    properties["course_axis_source_name"] = spec.name
    target_length = float(course_length or spec.width)
    properties["course_axis_length_m"] = target_length
    properties["course_axis_length_source"] = (
        "assembly_parameter" if course_length is not None else "source_width"
    )
    supported_blends = {"linear", "c1_smoothstep", "c1_two_block"}
    if blend_mode not in supported_blends:
        raise ValueError(
            f"Unsupported course-axis blend {blend_mode}; "
            f"expected one of {sorted(supported_blends)}"
        )
    properties["course_axis_blend_mode"] = blend_mode
    if blend_mode == "c1_two_block":
        if "seam_lateral_normalized" not in spec.properties:
            raise ValueError(
                "c1_two_block requires seam_lateral_normalized on the source"
            )
        properties["course_axis_knot_fraction"] = 0.5 * (
            float(spec.properties["seam_lateral_normalized"]) + 1.0
        )
    return ModuleSpec(
        name=name or f"{spec.name}_COURSE",
        page=spec.page,
        shape=f"course_axis:{spec.shape}",
        length=target_length,
        width=spec.length,
        inlet=np.array([spec.inlet[0], spec.outlet[0]], dtype=float),
        outlet=np.array([spec.inlet[1], spec.outlet[1]], dtype=float),
        properties=properties,
    )
