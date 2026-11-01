"""Validate the delivered robot assets and the combined PK-3A MJCF scene."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

import mujoco
import numpy as np

from .generate_closed_loop import default_output_root


WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def validate(scene_path: Path, metadata_path: Path) -> Dict[str, Any]:
    scene_path = Path(scene_path).resolve()
    metadata_path = Path(metadata_path).resolve()
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    failures = []
    expected_joints = ["floating_base_joint"] + [
        f"{name}_wheel_joint" for name in WHEELS
    ]
    missing_joints = [
        name for name in expected_joints
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) < 0
    ]
    if missing_joints:
        failures.append(f"missing joints: {missing_joints}")

    robot_mesh_names = ["robot_base_visual"] + [
        f"robot_wheel_{name}_visual" for name in WHEELS
    ]
    mesh_faces = 0
    for name in robot_mesh_names:
        mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, name)
        if mesh_id < 0:
            failures.append(f"missing visual mesh: {name}")
        else:
            mesh_faces += int(model.mesh_facenum[mesh_id])
    if mesh_faces != int(metadata["exported_visual_face_count"]):
        failures.append(
            f"robot visual faces {mesh_faces} != metadata "
            f'{metadata["exported_visual_face_count"]}'
        )

    robot_visual_geoms = ["robot_base_visual_geom"] + [
        f"{name}_wheel_visual_geom" for name in WHEELS
    ]
    for name in robot_visual_geoms:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom_id < 0:
            failures.append(f"missing visual geom: {name}")
        elif model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
            failures.append(f"visual mesh has collision enabled: {name}")

    wheel_geom_ids = {}
    for name in WHEELS:
        geom_id = model.geom(f"{name}_wheel_collision").id
        wheel_geom_ids[geom_id] = name
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            failures.append(f"wheel collision is not an ellipsoid: {name}")
    for name in ("chassis", "arm_pedestal", "arm_envelope"):
        geom_id = model.geom(f"robot_{name}_collision").id
        if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            failures.append(f"base collision is not a box: {name}")

    initial_robot_contacts = []
    track_geom_ids = {
        index for index in range(model.ngeom)
        if (model.geom(index).name or "").startswith("track_")
    }
    robot_collision_ids = set(wheel_geom_ids)
    robot_collision_ids.update(
        model.geom(f"robot_{name}_collision").id
        for name in ("chassis", "arm_pedestal", "arm_envelope")
    )
    for contact in data.contact[:data.ncon]:
        pair = {int(contact.geom1), int(contact.geom2)}
        if pair.intersection(robot_collision_ids):
            initial_robot_contacts.append(contact)
            if not pair.intersection(set(wheel_geom_ids)):
                failures.append("base/arm collision touches track at initial pose")
    maximum_initial_penetration = max(
        [-float(contact.dist) for contact in initial_robot_contacts] + [0.0]
    )
    if maximum_initial_penetration > 0.0002:
        failures.append(
            f"initial penetration {maximum_initial_penetration:.9f} m exceeds 0.2 mm"
        )

    base_id = model.body("base_link").id
    initial_position = data.xpos[base_id].copy()
    contact_seen = {name: False for name in WHEELS}
    finite = True
    peak_contact_count = data.ncon
    settling_steps = int(round(2.0 / model.opt.timestep))
    hold_steps = int(round(10.0 / model.opt.timestep))
    for step in range(hold_steps):
        mujoco.mj_step(model, data)
        finite = finite and bool(np.isfinite(data.qpos).all())
        finite = finite and bool(np.isfinite(data.qvel).all())
        peak_contact_count = max(peak_contact_count, data.ncon)
        for contact in data.contact[:data.ncon]:
            pair = {int(contact.geom1), int(contact.geom2)}
            if not pair.intersection(track_geom_ids):
                continue
            for geom_id, name in wheel_geom_ids.items():
                if geom_id in pair:
                    contact_seen[name] = True
        if step + 1 == settling_steps:
            settled_position = data.xpos[base_id].copy()
    if not finite:
        failures.append("non-finite simulation state")
    missing_contacts = [name for name, seen in contact_seen.items() if not seen]
    if missing_contacts:
        failures.append(f"wheels without track contact: {missing_contacts}")

    settling_distance = float(np.linalg.norm(settled_position - initial_position))
    if settling_distance > 0.15:
        failures.append(
            f"uncontrolled 2 s settling distance is {settling_distance:.6f} m"
        )
    hold_position = data.xpos[base_id].copy()
    hold_delta = hold_position - initial_position
    planar_drift = float(np.linalg.norm(hold_delta[:2]))
    if planar_drift > 0.02:
        failures.append(
            f"zero-command 10 s planar drift is {planar_drift:.6f} m"
        )

    products = set(metadata["step_inventory"]["products"])
    required_products = {
        "[3D]HBLS2D02E0200-D206",
        "86-0101 REV B ENVELOPE, VLP-16",
        "RealSense_D435",
        "MD200T, DMD200T.step",
        "robot_arm",
    }
    missing_products = sorted(required_products - products)
    if missing_products:
        failures.append(f"STEP inventory missing products: {missing_products}")

    return {
        "passed": not failures,
        "failures": failures,
        "scene": str(scene_path),
        "model": {
            "bodies": int(model.nbody),
            "joints": int(model.njnt),
            "actuators": int(model.nu),
            "geoms": int(model.ngeom),
            "mesh_faces_total": int(model.nmeshface),
            "robot_visual_faces": mesh_faces,
        },
        "step": {
            "products": metadata["step_inventory"]["product_count"],
            "assembly_occurrences": metadata["step_inventory"][
                "assembly_occurrence_count"
            ],
            "manifold_solids": metadata["step_inventory"][
                "manifold_solid_brep_count"
            ],
        },
        "contact": {
            "initial_robot_contact_count": len(initial_robot_contacts),
            "maximum_initial_penetration_m": maximum_initial_penetration,
            "peak_contact_count": int(peak_contact_count),
            "all_wheels_touched_track": all(contact_seen.values()),
        },
        "dynamics": {
            "finite": finite,
            "settling_distance_2s_m": settling_distance,
            "zero_command_planar_drift_10s_m": planar_drift,
            "zero_command_vertical_settling_10s_m": float(abs(hold_delta[2])),
            "initial_base_position_m": np.round(initial_position, 9).tolist(),
            "settled_base_position_2s_m": np.round(
                settled_position, 9
            ).tolist(),
            "hold_base_position_10s_m": np.round(hold_position, 9).tolist(),
        },
        "limitations": [
            "Arm joints remain locked pending STEP joint-axis audit.",
            "Base and wheel masses are provisional until measured.",
            "Wheel collision is an efficient ellipsoid, not the visual tread mesh.",
        ],
    }


def main() -> None:
    root = default_output_root()
    parser = argparse.ArgumentParser(description="Validate PK-3A robot scene")
    parser.add_argument(
        "--scene", type=Path, default=root / "models" / "pk3a_robot_scene.xml"
    )
    parser.add_argument(
        "--metadata",
        type=Path,
        default=root / "assets" / "robot" / "robot_metadata.json",
    )
    args = parser.parse_args()
    result = validate(args.scene, args.metadata)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["passed"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
