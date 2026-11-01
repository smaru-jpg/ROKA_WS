from pathlib import Path

import numpy as np

from roka_mujoco.module_geometry import load_modules, rasterize_module, surface_height


CONFIG = Path(__file__).resolve().parents[1] / "config" / "pk3a_modules.json"


def _edges(module, progress):
    right = surface_height(
        module,
        progress,
        np.full_like(progress, -0.5 * module.width),
    )
    left = surface_height(
        module,
        progress,
        np.full_like(progress, 0.5 * module.width),
    )
    return right, left


def test_final_3ta1l_uses_the_approved_spline_shape():
    modules = load_modules(CONFIG)
    assert modules["3TA1L"].shape == "ta1l_asymmetric_spline_loft"


def test_one_side_is_rectangular_and_the_other_is_a_spline():
    module = load_modules(CONFIG)["3TA1L"]
    progress = np.linspace(0.0, module.length, 401)
    rectangular, spline = _edges(module, progress)
    assert np.allclose(rectangular, 0.3883, atol=1.0e-12)
    assert np.isclose(spline[0], 0.0835)
    assert np.isclose(spline[-1], 0.6162)
    assert np.all(np.diff(spline) > 0.0)


def test_projected_side_areas_are_intentionally_different():
    module = load_modules(CONFIG)["3TA1L"]
    progress = np.linspace(0.0, module.length, 4001)
    rectangular, spline = _edges(module, progress)
    rectangular_area = np.trapz(rectangular, progress)
    spline_area = np.trapz(spline, progress)
    assert np.isclose(rectangular_area, 0.9144 * 0.3883, atol=1.0e-9)
    assert abs(rectangular_area - spline_area) > 0.001


def test_spline_endpoint_tangents_follow_the_drawing():
    module = load_modules(CONFIG)["3TA1L"]
    step = 1.0e-6
    _, start = _edges(module, np.array([0.0, step]))
    _, end = _edges(module, np.array([module.length - step, module.length]))
    start_angle = np.rad2deg(np.arctan2(start[1] - start[0], step))
    end_angle = np.rad2deg(np.arctan2(end[1] - end[0], step))
    assert np.isclose(start_angle, 7.2, atol=0.05)
    assert np.isclose(end_angle, 38.8, atol=0.05)


def test_spline_has_cnc_straight_segments_matching_low_angle_tb_edge():
    module = load_modules(CONFIG)["3TA1L"]
    progress = np.linspace(0.0, module.length, 4001)
    _, spline = _edges(module, progress)
    derivative = np.gradient(spline, progress)
    start = progress < 0.9 * 0.12 * module.length
    end = progress > (1.0 - 0.9 * 0.10) * module.length
    assert np.ptp(derivative[start]) < 1.0e-6
    assert np.ptp(derivative[end]) < 1.0e-6


def test_loft_is_finite_monotonic_and_full_width():
    module = load_modules(CONFIG)["3TA1L"]
    raster, coordinates = rasterize_module(module, 0.008)
    assert raster.shape == (115, 115)
    assert coordinates["inside"].all()
    assert np.isfinite(raster).all()
    assert np.isclose(raster.min(), 0.0835)
    assert np.isclose(raster.max(), 0.6162)
