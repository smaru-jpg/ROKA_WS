"""Validate geometry, ground support and contacts for PK-3A S00-S03."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import mujoco
import numpy as np

from .module_geometry import load_json


def _ray_height(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_id: int,
    x: float,
    y: float,
) -> float:
    origin_z = 2.5
    distance = mujoco.mj_rayHfield(
        model,
        data,
        geom_id,
        np.array([x, y, origin_z], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
        None,
    )
    return float("nan") if distance < 0.0 else origin_z - float(distance)


def _drop_all_seams(model_path: Path, seams: List[Dict[str, Any]]) -> Dict[str, Any]:
    spec = mujoco.MjSpec.from_file(str(model_path))
    for seam in seams:
        point = np.asarray(seam["points_xyz_m"], dtype=float)[50]
        body = spec.worldbody.add_body(
            name=f'probe_body_{seam["index"]:02d}',
            pos=[float(point[0]), float(point[1]), float(point[2] + 0.35)],
        )
        body.add_freejoint(name=f'probe_joint_{seam["index"]:02d}')
        body.add_geom(
            name=f'probe_geom_{seam["index"]:02d}',
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.035, 0.0, 0.0],
            mass=0.35,
            friction=[1.0, 0.01, 0.001],
        )
    model = spec.compile()
    data = mujoco.MjData(model)
    track_ids = {
        model.geom(index).id
        for index in range(model.ngeom)
        if model.geom(index).name.startswith("track_")
    }
    probe_ids = {
        model.geom(f'probe_geom_{seam["index"]:02d}').id: seam["index"]
        for seam in seams
    }
    contact_seen = {seam["index"]: False for seam in seams}
    finite = True
    for _ in range(int(round(1.3 / model.opt.timestep))):
        mujoco.mj_step(model, data)
        finite = finite and bool(np.isfinite(data.qpos).all())
        finite = finite and bool(np.isfinite(data.qvel).all())
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            pair = {int(contact.geom1), int(contact.geom2)}
            probe = next((value for key, value in probe_ids.items() if key in pair), None)
            if probe is not None and pair.intersection(track_ids):
                contact_seen[probe] = True
    missing = [index for index, seen in contact_seen.items() if not seen]
    return {
        "finite": finite,
        "contact_count": sum(contact_seen.values()),
        "probe_count": len(seams),
        "missing_seam_indices": missing,
    }


def _surface_material_probe(
    model_path: Path,
    expected_contact: Dict[str, Any],
) -> Dict[str, Any]:
    """Drop a tire-like contact probe on the flat riding surface.

    This checks the compiled hfield contact, not only the XML attributes.  The
    probe intentionally has lower priority and lower friction so the track's
    priority-1 material profile must determine the resulting contact.
    """
    base_model = mujoco.MjModel.from_xml_path(str(model_path))
    base_data = mujoco.MjData(base_model)
    mujoco.mj_forward(base_model, base_data)
    flat_name = "track_S04_F_HALF"
    flat_geom = base_model.geom(flat_name).id
    x, y = base_data.geom_xpos[flat_geom, :2]
    surface_z = _ray_height(base_model, base_data, flat_geom, float(x), float(y))
    if not np.isfinite(surface_z):
        return {
            "finite": False,
            "contact_count": 0,
            "detail": f"ray missed {flat_name}",
        }

    spec = mujoco.MjSpec.from_file(str(model_path))
    body = spec.worldbody.add_body(
        name="surface_material_probe_body",
        pos=[float(x), float(y), float(surface_z + 0.25)],
    )
    body.add_freejoint(name="surface_material_probe_joint")
    body.add_geom(
        name="surface_material_probe_geom",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.04, 0.0, 0.0],
        mass=0.5,
        friction=[0.6, 0.003, 0.0001],
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    track_ids = {
        model.geom(index).id
        for index in range(model.ngeom)
        if model.geom(index).name.startswith("track_")
    }
    probe_id = model.geom("surface_material_probe_geom").id
    contact_dims = set()
    maximum_friction_error = 0.0
    minimum_distance = 0.0
    contact_count = 0
    finite = True
    friction = np.asarray(expected_contact["friction"], dtype=float)
    expected_pair_friction = np.array([
        friction[0],
        friction[0],
        friction[1],
        friction[2],
        friction[2],
    ])
    for _ in range(int(round(1.0 / model.opt.timestep))):
        mujoco.mj_step(model, data)
        finite = finite and bool(np.isfinite(data.qpos).all())
        finite = finite and bool(np.isfinite(data.qvel).all())
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            pair = {int(contact.geom[0]), int(contact.geom[1])}
            if probe_id not in pair or not pair.intersection(track_ids):
                continue
            contact_count += 1
            contact_dims.add(int(contact.dim))
            minimum_distance = min(minimum_distance, float(contact.dist))
            maximum_friction_error = max(
                maximum_friction_error,
                float(np.max(np.abs(
                    np.asarray(contact.friction) - expected_pair_friction
                ))),
            )
    return {
        "finite": finite,
        "contact_count": contact_count,
        "contact_dims": sorted(contact_dims),
        "expected_pair_friction": expected_pair_friction.tolist(),
        "maximum_friction_error": maximum_friction_error,
        "minimum_contact_distance_m": minimum_distance,
        "final_speed_m_s": float(np.linalg.norm(data.qvel[-6:-3])),
    }


def validate(config_path: Path, output_root: Path) -> Dict[str, Any]:
    config = load_json(config_path)
    expected = config["expected"]
    artifact_stem = str(config.get("artifact_stem", "pk3a_closed_loop"))
    metadata_path = output_root / "assets" / f"{artifact_stem}_metadata.json"
    model_path = output_root / "models" / f"{artifact_stem}.xml"
    metadata = load_json(metadata_path)
    checks: List[Dict[str, Any]] = []

    def add(name: str, passed: bool, detail: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    add(
        "instance_count",
        metadata["instance_count"] == expected["instance_count"],
        f'count={metadata["instance_count"]}',
    )
    add(
        "seam_count",
        metadata["seam_count"] == expected["seam_count"],
        f'count={metadata["seam_count"]}',
    )
    add(
        "module_counts",
        metadata["module_counts"] == expected["module_counts"],
        json.dumps(metadata["module_counts"], sort_keys=True),
    )
    if "start_zone_instance_id" in expected:
        start_zone = metadata.get("start_zone")
        expected_start = str(expected["start_zone_instance_id"])
        add(
            "start_zone_assignment",
            start_zone is not None
            and start_zone.get("line_instance_id") == expected_start
            and expected_start in start_zone.get("instance_ids", []),
            "missing" if start_zone is None else json.dumps(
                start_zone,
                sort_keys=True,
            ),
        )
    analytic_height = float(metadata["maximum_analytic_seam_height_error_m"])
    analytic_plan = float(metadata["maximum_analytic_seam_plan_error_m"])
    add(
        "analytic_seam_height",
        analytic_height < float(expected["maximum_seam_height_error_m"]),
        f"maximum={analytic_height:.9f} m",
    )
    add(
        "analytic_seam_plan",
        analytic_plan <= 0.005,
        f"maximum={analytic_plan:.9f} m",
    )
    if "maximum_course_tangent_error_deg" in expected:
        tangent_error = float(metadata["maximum_course_tangent_error_deg"])
        add(
            "course_tangent_continuity",
            tangent_error
            <= float(expected["maximum_course_tangent_error_deg"]),
            f"maximum={tangent_error:.6f} deg",
        )
    for module_name, limit in expected.get(
        "maximum_longitudinal_slope_deg_by_module", {}
    ).items():
        observed = float(
            metadata["module_slope_metrics"][module_name][
                "maximum_longitudinal_slope_deg"
            ]
        )
        add(
            f"longitudinal_slope_{module_name}",
            observed <= float(limit),
            f"maximum={observed:.6f} deg, limit={float(limit):.6f} deg",
        )
    add(
        "cv_turn",
        np.isclose(metadata["cv_boundary_turn_deg"], expected["cv_signed_turn_deg"], atol=1.0e-5),
        f'boundary_turn={metadata["cv_boundary_turn_deg"]:.6f} deg',
    )
    if "cv_group_turns_deg" in expected:
        observed_groups = np.asarray(metadata["cv_group_turns_deg"], dtype=float)
        expected_groups = np.asarray(expected["cv_group_turns_deg"], dtype=float)
        add(
            "cv_group_turns",
            observed_groups.shape == expected_groups.shape
            and np.allclose(observed_groups, expected_groups, atol=1.0e-5),
            f"groups={observed_groups.tolist()}",
        )
    maximum_axis_angle = float(metadata["maximum_adjacent_axis_angle_deg"])
    add(
        "no_perpendicular_module_joint",
        maximum_axis_angle <= float(expected["maximum_adjacent_axis_angle_deg"]),
        f"maximum_axis_angle={maximum_axis_angle:.6f} deg",
    )
    overlap_area = float(metadata["maximum_nonadjacent_overlap_area_m2"])
    add(
        "nonadjacent_footprint_overlap",
        metadata["nonadjacent_overlap_count"] == 0
        and overlap_area <= float(expected["maximum_nonadjacent_overlap_m2"]),
        f'matches={metadata["nonadjacent_overlap_count"]}, maximum={overlap_area:.9f} m^2',
    )
    if "maximum_closure_position_error_m" in expected:
        closure = metadata.get("closure")
        add(
            "closed_loop_position",
            bool(metadata.get("closed_loop"))
            and closure is not None
            and float(closure["position_error_m"])
            <= float(expected["maximum_closure_position_error_m"]),
            "missing" if closure is None else f'error={closure["position_error_m"]:.9f} m',
        )
        add(
            "closed_loop_yaw",
            closure is not None
            and float(closure["yaw_error_deg"])
            <= float(expected["maximum_closure_yaw_error_deg"]),
            "missing" if closure is None else f'error={closure["yaw_error_deg"]:.9f} deg',
        )

    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        track_ids = {
            item["id"]: model.geom(f'track_{item["id"]}').id
            for item in metadata["instances"]
        }
        add(
            "mjcf_load",
            len(track_ids) == expected["instance_count"]
            and model.nhfield == len(metadata["asset_rasters"]),
            f"track_geoms={len(track_ids)}, nhfield={model.nhfield}, ngeom={model.ngeom}",
        )
        module_solids = metadata.get("module_solids", {})
        if module_solids:
            visual_ids = [
                model.geom(index).id
                for index in range(model.ngeom)
                if model.geom(index).name.startswith("visual_")
            ]
            solids_valid = all(
                bool(value["watertight"])
                and bool(value["winding_consistent"])
                and int(value["visual_face_count"]) > 0
                and float(value["volume_m3"]) > 0.0
                for value in module_solids.values()
            )
            visual_only = all(
                model.geom_contype[index] == 0
                and model.geom_conaffinity[index] == 0
                for index in visual_ids
            )
            add(
                "modular_watertight_visual_solids",
                solids_valid
                and model.nmesh == len(module_solids)
                and len(visual_ids) == expected["instance_count"]
                and visual_only,
                f"solids={len(module_solids)}, nmesh={model.nmesh}, "
                f"visual_instances={len(visual_ids)}, visual_only={visual_only}",
            )
            collision_hidden = all(
                model.geom_group[index] == 4
                and np.isclose(model.geom_rgba[index, 3], 0.0)
                for index in track_ids.values()
            )
            add(
                "hidden_collision_layer",
                collision_hidden,
                f"hidden={collision_hidden}, collision_instances={len(track_ids)}",
            )
            maximum_faces = int(expected.get(
                "maximum_expanded_visual_faces", 500_000
            ))
            expanded_faces = int(metadata["expanded_visual_face_count"])
            add(
                "visual_triangle_budget",
                expanded_faces <= maximum_faces,
                f"faces={expanded_faces}, limit={maximum_faces}",
            )
            module_root = model_path.parent / "modules"
            standalone_failures = []
            for name in module_solids:
                try:
                    standalone = mujoco.MjModel.from_xml_path(
                        str(module_root / f"{name}.xml")
                    )
                    if standalone.nmesh != 1 or standalone.nhfield != 1:
                        standalone_failures.append(
                            f"{name}:nmesh={standalone.nmesh},nhfield={standalone.nhfield}"
                        )
                except Exception as error:
                    standalone_failures.append(f"{name}:{error}")
            add(
                "standalone_module_models",
                not standalone_failures,
                "failures=none" if not standalone_failures else "; ".join(
                    standalone_failures
                ),
            )
        surface_contact = metadata.get("surface_contact")
        if surface_contact is not None:
            ids = np.asarray(list(track_ids.values()), dtype=int)
            expected_friction = np.asarray(
                surface_contact["friction"], dtype=float
            )
            expected_solref = np.asarray(surface_contact["solref"], dtype=float)
            expected_solimp = np.asarray(surface_contact["solimp"], dtype=float)
            contact_compiled = (
                np.all(model.geom_condim[ids] == int(surface_contact["condim"]))
                and np.all(model.geom_priority[ids] == int(
                    surface_contact["priority"]
                ))
                and np.allclose(model.geom_solmix[ids], surface_contact["solmix"])
                and np.allclose(model.geom_friction[ids], expected_friction)
                and np.allclose(model.geom_solref[ids], expected_solref)
                and np.allclose(model.geom_solimp[ids], expected_solimp)
            )
            add(
                "surface_material_contact_parameters",
                contact_compiled,
                "friction=" + np.array2string(model.geom_friction[ids[0]])
                + f", condim={model.geom_condim[ids[0]]}, "
                + "solref=" + np.array2string(model.geom_solref[ids[0]])
                + ", solimp=" + np.array2string(model.geom_solimp[ids[0]]),
            )
        contact_solver = metadata.get("contact_solver")
        if contact_solver is not None:
            multiccd_enabled = bool(
                model.opt.enableflags
                & int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
            )
            solver_compiled = (
                model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
                and model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON
                and model.opt.iterations == int(contact_solver["iterations"])
                and np.isclose(model.opt.tolerance, contact_solver["tolerance"])
                and np.isclose(model.opt.impratio, contact_solver["impratio"])
                and multiccd_enabled == bool(contact_solver["multiccd"])
            )
            add(
                "surface_material_contact_solver",
                solver_compiled,
                f"cone={model.opt.cone}, solver={model.opt.solver}, "
                f"iterations={model.opt.iterations}, impratio={model.opt.impratio}, "
                f"multiccd={multiccd_enabled}",
            )
        if "start_zone_instance_id" in expected:
            start_line_ids = [
                model.geom(index).id
                for index in range(model.ngeom)
                if model.geom(index).name.startswith("start_line_")
            ]
            visual_only = all(
                model.geom_contype[index] == 0
                and model.geom_conaffinity[index] == 0
                for index in start_line_ids
            )
            add(
                "start_zone_visual",
                bool(start_line_ids) and visual_only,
                f"visual_geoms={len(start_line_ids)}, visual_only={visual_only}",
            )
    except Exception as error:
        add("mjcf_load", False, str(error))
        return {"passed": False, "checks": checks}

    bottom_values = []
    for geom_id in track_ids.values():
        hfield_id = model.geom_dataid[geom_id]
        bottom_values.append(
            float(model.geom_pos[geom_id, 2] - model.hfield_size[hfield_id, 3])
        )
    add(
        "ground_support",
        max(bottom_values) <= -0.005 + 1.0e-9,
        f"highest_bottom={max(bottom_values):.6f} m",
    )

    ray_pair_errors = []
    ray_expected_errors = []
    missing_rays = 0
    ray_inset = float(config["generation"].get("collision_ray_inset_m", 0.012))
    for seam in metadata["seams"]:
        from_id = track_ids[seam["from_instance"]]
        to_id = track_ids[seam["to_instance"]]
        points = np.asarray(seam["points_xyz_m"], dtype=float)
        from_offset = ray_inset * np.asarray(
            seam["target_inward_direction_xy"], dtype=float
        )
        to_offset = ray_inset * np.asarray(
            seam["source_inward_direction_xy"], dtype=float
        )
        for point in points[10:-10:20]:
            from_xy = point[:2] + from_offset
            to_xy = point[:2] + to_offset
            from_height = _ray_height(model, data, from_id, from_xy[0], from_xy[1])
            to_height = _ray_height(model, data, to_id, to_xy[0], to_xy[1])
            if not np.isfinite(from_height) or not np.isfinite(to_height):
                missing_rays += 1
                continue
            ray_pair_errors.append(abs(from_height - to_height))
            ray_expected_errors.extend([
                abs(from_height - point[2]),
                abs(to_height - point[2]),
            ])
    maximum_pair = max(ray_pair_errors, default=float("inf"))
    maximum_expected = max(ray_expected_errors, default=float("inf"))
    add(
        "collision_seam_rays",
        missing_rays == 0 and maximum_pair < 0.01 and maximum_expected < 0.01,
        f"inset={ray_inset:.6f} m, missing={missing_rays}, pair_max={maximum_pair:.6f} m, expected_max={maximum_expected:.6f} m",
    )

    try:
        drop = _drop_all_seams(model_path, metadata["seams"])
        add(
            "all_seam_drop_contacts",
            drop["finite"] and not drop["missing_seam_indices"],
            json.dumps(drop, sort_keys=True),
        )
    except Exception as error:
        add("all_seam_drop_contacts", False, str(error))

    if metadata.get("surface_contact") is not None:
        try:
            probe = _surface_material_probe(
                model_path,
                metadata["surface_contact"],
            )
            expected_dim = int(metadata["surface_contact"]["condim"])
            add(
                "surface_material_dynamic_probe",
                probe["finite"]
                and probe["contact_count"] > 0
                and probe["contact_dims"] == [expected_dim]
                and probe["maximum_friction_error"] < 1.0e-12
                and probe["minimum_contact_distance_m"] > -0.005,
                json.dumps(probe, sort_keys=True),
            )
        except Exception as error:
            add("surface_material_dynamic_probe", False, str(error))

    return {
        "passed": all(check["passed"] for check in checks),
        "checks": checks,
    }
