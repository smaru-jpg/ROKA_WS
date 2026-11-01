"""Watertight visual solids and coarse collision fields for PK-3A modules."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import trimesh

from .module_geometry import (
    ModuleSpec,
    module_bounds,
    rasterize_module,
    surface_height,
    surface_point,
)


GROUND_Z = -0.032
DEFAULT_VISUAL_SPACING_M = 0.040
DEFAULT_COLLISION_SPACING_M = 0.005


def _grid_count(span: float, spacing: float) -> int:
    if spacing <= 0.0:
        raise ValueError(f"Grid spacing must be positive, got {spacing}")
    return max(3, int(np.ceil(float(span) / float(spacing))) + 1)


def _surface_grid(
    spec: ModuleSpec,
    spacing: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return progress/lateral samples and XYZ vertices in module coordinates."""
    progress = np.linspace(0.0, spec.length, _grid_count(spec.length, spacing))
    lateral = np.linspace(
        -0.5 * spec.width,
        0.5 * spec.width,
        _grid_count(spec.width, spacing),
    )
    xyz = np.empty((len(progress), len(lateral), 3), dtype=float)
    for row, distance in enumerate(progress):
        for column, offset in enumerate(lateral):
            xyz[row, column, :2] = surface_point(
                spec, float(distance), float(offset)
            )
    pp, ll = np.meshgrid(progress, lateral, indexing="ij")
    xyz[:, :, 2] = surface_height(spec, pp, ll)
    return progress, lateral, xyz


def build_watertight_visual_mesh(
    spec: ModuleSpec,
    *,
    spacing: float = DEFAULT_VISUAL_SPACING_M,
    bottom_z: float = GROUND_Z,
) -> trimesh.Trimesh:
    """Build one closed solid from a module's analytic riding surface.

    The top and bottom use identical structured topology. Connecting their
    four boundary rings produces a deterministic, watertight module without a
    CAD boolean engine. Curved CV modules use ``surface_point`` so the solid
    follows the annular footprint instead of occupying its bounding rectangle.
    """
    _, _, top_grid = _surface_grid(spec, spacing)
    rows, columns = top_grid.shape[:2]
    top = top_grid.reshape((-1, 3))
    bottom = top.copy()
    bottom[:, 2] = float(bottom_z)
    vertices = np.vstack((top, bottom))
    bottom_offset = len(top)

    def vertex(row: int, column: int) -> int:
        return row * columns + column

    faces = []
    for row in range(rows - 1):
        for column in range(columns - 1):
            a = vertex(row, column)
            b = vertex(row + 1, column)
            c = vertex(row + 1, column + 1)
            d = vertex(row, column + 1)
            faces.extend(((a, b, c), (a, c, d)))
            faces.extend((
                (bottom_offset + a, bottom_offset + c, bottom_offset + b),
                (bottom_offset + a, bottom_offset + d, bottom_offset + c),
            ))

    boundaries = (
        [(row, 0) for row in range(rows)],
        [(rows - 1, column) for column in range(columns)],
        [(row, columns - 1) for row in range(rows - 1, -1, -1)],
        [(0, column) for column in range(columns - 1, -1, -1)],
    )
    for boundary in boundaries:
        for first, second in zip(boundary[:-1], boundary[1:]):
            a = vertex(*first)
            b = vertex(*second)
            faces.extend((
                (a, bottom_offset + b, b),
                (a, bottom_offset + a, bottom_offset + b),
            ))

    mesh = trimesh.Trimesh(
        vertices=vertices,
        faces=np.asarray(faces, dtype=np.int64),
        process=True,
        validate=True,
    )
    trimesh.repair.fix_normals(mesh, multibody=False)
    mesh.remove_unreferenced_vertices()
    if mesh.volume < 0.0:
        mesh.invert()
    if not mesh.is_watertight:
        raise ValueError(f"Generated module {spec.name} is not watertight")
    if not mesh.is_winding_consistent:
        raise ValueError(f"Generated module {spec.name} has inconsistent winding")
    if not np.isfinite(mesh.vertices).all():
        raise ValueError(f"Generated module {spec.name} has non-finite vertices")
    return mesh


def build_collision_field(
    spec: ModuleSpec,
    *,
    spacing: float = DEFAULT_COLLISION_SPACING_M,
    outside_height: float = GROUND_Z,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Build a coarse hidden collision field.

    A heightfield is rectangular. Samples outside a curved module's true
    footprint are therefore lowered to the physical floor rather than left as
    a raised rectangular apron.
    """
    height, coordinates = rasterize_module(spec, spacing)
    inside = np.asarray(coordinates["inside"], dtype=bool)
    height = np.asarray(height, dtype=float)
    outside = ~inside
    halo_count = 0
    if outside.any():
        dy = float(abs(coordinates["y"][0] - coordinates["y"][-1]) /
                   (height.shape[0] - 1))
        dx = float(abs(coordinates["x"][-1] - coordinates["x"][0]) /
                   (height.shape[1] - 1))
        distance, nearest = distance_transform_edt(
            outside,
            sampling=(dy, dx),
            return_indices=True,
        )
        halo = outside & (distance <= max(0.02, 2.0 * spacing))
        height[halo] = height[nearest[0, halo], nearest[1, halo]]
        height[outside & ~halo] = float(outside_height)
        halo_count = int(halo.sum())
    minimum = float(height.min())
    maximum = float(height.max())
    height_range = max(maximum - minimum, 1.0e-6)
    bounds = module_bounds(spec)
    records = {
        "minimum": minimum,
        "maximum": maximum,
        "height_range": height_range,
        "base_depth": max(0.03, minimum - float(outside_height) + 0.03),
        "bounds": bounds,
        "bounds_centre": np.array([
            0.5 * (bounds[0] + bounds[1]),
            0.5 * (bounds[2] + bounds[3]),
        ]),
        "raster_shape": list(height.shape),
        "actual_cell_size_m": [
            float(abs(coordinates["y"][0] - coordinates["y"][-1]) /
                  (height.shape[0] - 1)),
            float(abs(coordinates["x"][-1] - coordinates["x"][0]) /
                  (height.shape[1] - 1)),
        ],
        "outside_sample_count": int((~inside).sum()),
        "collision_halo_sample_count": halo_count,
    }
    return height, records


def write_module_assets(
    name: str,
    spec: ModuleSpec,
    visual_root: Path,
    collision_root: Path,
    *,
    visual_spacing: float = DEFAULT_VISUAL_SPACING_M,
    collision_spacing: float = DEFAULT_COLLISION_SPACING_M,
    ground_z: float = GROUND_Z,
) -> Dict[str, Any]:
    """Write one visual OBJ and one collision PNG, returning MJCF metadata."""
    visual_root.mkdir(parents=True, exist_ok=True)
    collision_root.mkdir(parents=True, exist_ok=True)

    mesh = build_watertight_visual_mesh(
        spec, spacing=visual_spacing, bottom_z=ground_z
    )
    visual_path = visual_root / f"{name}.obj"
    visual_path.write_text(
        trimesh.exchange.obj.export_obj(
            mesh,
            include_normals=False,
            include_color=False,
            include_texture=False,
            header=None,
        ),
        encoding="utf-8",
    )

    height, collision = build_collision_field(
        spec, spacing=collision_spacing, outside_height=ground_z
    )
    normalized = np.rint(
        np.clip(
            (height - collision["minimum"]) / collision["height_range"],
            0.0,
            1.0,
        )
        * 65535.0
    ).astype(np.uint16)
    collision_path = collision_root / f"{name}.png"
    Image.fromarray(normalized).save(collision_path)

    return {
        **collision,
        "visual_path": str(visual_path),
        "collision_path": str(collision_path),
        "visual_vertex_count": int(len(mesh.vertices)),
        "visual_face_count": int(len(mesh.faces)),
        "visual_watertight": bool(mesh.is_watertight),
        "visual_winding_consistent": bool(mesh.is_winding_consistent),
        "visual_volume_m3": float(mesh.volume),
        "visual_bounds_xyz_m": mesh.bounds.tolist(),
    }
