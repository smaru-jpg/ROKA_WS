from pathlib import Path

import numpy as np

from roka_mujoco.module_geometry import (
    mirror_module_y,
    orient_course_across_width,
    rasterize_module,
    surface_height,
    surface_point,
)
from roka_mujoco.port_geometry import load_port_sources, sample_port


CONFIG = Path(__file__).resolve().parents[1] / "config" / "pk3a_port_sources.json"


def test_y_mirror_is_metric_and_does_not_warp_the_surface():
    _, modules = load_port_sources(CONFIG)
    source = modules["3TB1L"]
    mirrored = mirror_module_y(source, "3TB1L_M")
    progress = np.linspace(0.0, source.length, 97)
    lateral = np.linspace(-0.5 * source.width, 0.5 * source.width, 97)
    expected = surface_height(source, progress, -lateral)
    observed = surface_height(mirrored, progress, lateral)
    assert np.allclose(observed, expected, atol=1.0e-12)
    assert mirrored.length == source.length
    assert mirrored.width == source.width


def test_mirrored_curve_changes_turn_sign_and_reflects_plan_points():
    _, modules = load_port_sources(CONFIG)
    source = modules["3CV1"]
    mirrored = mirror_module_y(source, "3CV1_M")
    assert mirrored.properties["turn_deg"] == -source.properties["turn_deg"]
    for progress in np.linspace(0.0, source.length, 11):
        point = surface_point(source, progress, -0.2)
        reflected = surface_point(mirrored, progress, 0.2)
        assert np.allclose(reflected, [point[0], -point[1]], atol=1.0e-12)


def test_mirrored_raster_is_the_row_reflection_of_the_source():
    _, modules = load_port_sources(CONFIG)
    source = modules["3TA1L"]
    mirrored = mirror_module_y(source, "3TA1L_M")
    source_height, _ = rasterize_module(source, 0.012)
    mirror_height, _ = rasterize_module(mirrored, 0.012)
    assert np.allclose(mirror_height, source_height[::-1], atol=1.0e-12)


def test_mirrored_transition_ports_match_within_the_ten_mm_rule():
    _, modules = load_port_sources(CONFIG)
    ta = mirror_module_y(modules["3TA1L"], "3TA1L_M")
    tb = mirror_module_y(modules["3TB1L"], "3TB1L_M")
    cv = mirror_module_y(modules["3CV1"], "3CV1_M")
    ta_edge = np.asarray(sample_port(ta, "right_edge", 101)["height_profile_m"])
    tb_edge = np.asarray(sample_port(tb, "left_edge", 101)["height_profile_m"])
    tb_bank = np.asarray(sample_port(tb, "right_edge", 101)["height_profile_m"])
    cv_bank = np.asarray(sample_port(cv, "inlet", 101)["height_profile_m"])
    assert np.max(np.abs(ta_edge - tb_edge)) < 0.002
    assert np.max(np.abs(tb_bank - cv_bank)) < 0.003


def test_course_axis_adapter_turns_drawing_side_profiles_into_end_ports():
    _, modules = load_port_sources(CONFIG)
    ta = orient_course_across_width(modules["3TA1L"], "3TA1L_C")
    tb = orient_course_across_width(modules["3TB1L"], "3TB1L_C")
    assert np.allclose(ta.inlet, [0.3883, 0.3883])
    assert np.allclose(ta.outlet, [0.0835, 0.6162])
    assert np.allclose(tb.inlet, [0.0835, 0.6146])
    assert np.allclose(tb.outlet, [0.0835, 0.8432])
    assert np.max(np.abs(ta.outlet - tb.inlet)) < 0.002


def test_course_axis_adapter_only_reparameterizes_the_approved_surface():
    _, modules = load_port_sources(CONFIG)
    source = modules["3TA1L"]
    course = orient_course_across_width(source, "3TA1L_C")
    course_progress = np.linspace(0.0, course.length, 101)
    course_lateral = np.linspace(-0.5 * course.width, 0.5 * course.width, 101)
    observed = surface_height(course, course_progress, course_lateral)
    expected = surface_height(
        source,
        course_lateral + 0.5 * source.length,
        course_progress - 0.5 * source.width,
    )
    assert np.allclose(observed, expected, atol=1.0e-12)
