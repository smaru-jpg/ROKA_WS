from roka_mujoco.benchmark_runner import EpisodeMetrics, benchmark_decision


def _status(sim_time, lateral=0.05, recovery=False):
    return {
        "episode_elapsed_sim_sec": sim_time,
        "path_progress_pct": min(99.0, sim_time),
        "reference_lateral_error_m": lateral,
        "reference_heading_error_deg": 4.0,
        "roll_deg": 20.0,
        "pitch_deg": 25.0,
        "reference_height_delta_m": 0.05,
        "tire_clearance_m": 0.02,
        "recovery_count": int(recovery),
        "recovery_active": recovery,
        "corridor_available": not recovery,
        "command_linear_mps": 0.30,
        "command_angular_radps": 0.40,
    }


def test_episode_metrics_accumulate_recovery_and_errors():
    metrics = EpisodeMetrics(1)
    metrics.update(_status(0.0))
    metrics.update(_status(0.1, lateral=0.15, recovery=True))
    metrics.update(_status(0.2, lateral=0.04))
    row = metrics.finish(True, "completed_lap")

    assert row["success"] == 1
    assert row["max_progress_pct"] == 100.0
    assert row["max_abs_lateral_error_m"] == 0.15
    assert row["recovery_count"] == 1
    assert row["recovery_time_sec"] == 0.1


def test_benchmark_decision_requires_every_run_for_pass():
    passing = {
        "success": 1,
        "max_abs_lateral_error_m": 0.20,
        "max_abs_roll_deg": 45.0,
        "max_abs_pitch_deg": 40.0,
    }
    failing = {**passing, "success": 0}

    assert benchmark_decision([passing] * 10)["decision"] == "PASS"
    assert benchmark_decision([passing] * 9 + [failing])["decision"] == "CAUTION"
