import pytest

from roka_mission.blind_profile import (
    project_route_progress,
    route_length,
    sample_at,
    validate_profile,
)


def profile():
    return {
        "format": "roka_blind_v2",
        "entry_progress_ratio": 0.0,
        "exit_progress_ratio": 0.30,
        "route_length_m": 1.0,
        "duration_sec": 1.0,
        "distance_m": 0.2,
        "commands": [
            {"elapsed_sec": 0.0, "linear_mps": 0.1, "angular_radps": 0.0},
            {"elapsed_sec": 0.2, "linear_mps": 0.1, "angular_radps": 0.0},
            {"elapsed_sec": 0.4, "linear_mps": 0.1, "angular_radps": 0.0},
            {"elapsed_sec": 0.6, "linear_mps": 0.1, "angular_radps": 0.0},
            {"elapsed_sec": 0.8, "linear_mps": 0.1, "angular_radps": 0.0},
            {"elapsed_sec": 1.0, "linear_mps": 0.0, "angular_radps": 0.0},
        ],
    }


def test_profile_uses_zero_order_hold():
    validated = validate_profile(profile())
    assert sample_at(validated, 0.5)["linear_mps"] == 0.1
    assert sample_at(validated, 1.0)["linear_mps"] == 0.0


def test_profile_rejects_unsafe_command():
    document = profile()
    document["commands"][0]["linear_mps"] = 0.61
    with pytest.raises(ValueError, match="envelope"):
        validate_profile(document)


def test_profile_rejects_nonfinite_and_sparse_timing():
    document = profile()
    document["duration_sec"] = float("nan")
    with pytest.raises(ValueError, match="duration"):
        validate_profile(document)

    document = profile()
    document["commands"][1]["elapsed_sec"] = 0.30
    with pytest.raises(ValueError, match="command gap"):
        validate_profile(document)


def test_profile_rejects_invalid_distance_and_negative_time():
    document = profile()
    document["distance_m"] = float("inf")
    with pytest.raises(ValueError, match="distance"):
        validate_profile(document)

    document = profile()
    document["commands"][0]["elapsed_sec"] = -0.01
    with pytest.raises(ValueError, match="timestamps"):
        validate_profile(document)


def test_profile_requires_start_origin_and_supported_exit():
    document = profile()
    document["entry_progress_ratio"] = 0.01
    with pytest.raises(ValueError, match="must be 0.0"):
        validate_profile(document)

    document = profile()
    document["exit_progress_ratio"] = 0.25
    with pytest.raises(ValueError, match="0.20, 0.30 or 0.40"):
        validate_profile(document)


@pytest.mark.parametrize("exit_ratio", [0.20, 0.30, 0.40])
def test_all_operator_exit_choices_are_accepted(exit_ratio):
    document = profile()
    document["exit_progress_ratio"] = exit_ratio
    assert validate_profile(document)["exit_progress_ratio"] == exit_ratio


def test_route_projection_reports_global_progress_and_distance():
    points = [
        {"x": 0.0, "y": 0.0},
        {"x": 1.0, "y": 0.0},
        {"x": 2.0, "y": 0.0},
    ]
    assert route_length(points) == pytest.approx(2.0)
    progress, distance = project_route_progress(points, 0.6, 0.2)
    assert progress == pytest.approx(0.30)
    assert distance == pytest.approx(0.20)
