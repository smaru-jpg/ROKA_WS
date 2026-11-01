import numpy as np

from roka_mujoco.traversability_dataset import (
    _mark_sustained_stalls,
    _unwrap_progress,
    fit_monotone_risk_model,
    static_feature_vector,
)


def test_progress_detects_closed_lap():
    indices = list(range(800, 858)) + list(range(0, 801))
    assert _unwrap_progress(indices) > 0.99


def test_static_features_increase_with_speed_and_difficulty():
    easy = static_feature_vector(0.20, 2.0, 3.0, 0.05, 0.15)
    hard = static_feature_vector(0.50, 20.0, 25.0, 0.80, -0.01)
    assert np.all(hard >= easy)
    assert np.any(hard > easy)


def test_stall_label_requires_sustained_low_motion():
    rows = [
        {
            "time_sec": index * 0.1,
            "command_linear_mps": 0.30,
            "forward_speed_mps": 0.02,
        }
        for index in range(12)
    ]
    stalled = _mark_sustained_stalls(rows)
    assert np.all(stalled)
    rows = rows[:5]
    assert not np.any(_mark_sustained_stalls(rows))


def test_fitted_risk_model_is_monotone():
    rng = np.random.default_rng(5)
    x = rng.uniform(0.0, 1.0, size=(300, 9))
    labels = ((x[:, 0] + x[:, 2] + x[:, 5]) > 1.5).astype(float)
    model, metrics = fit_monotone_risk_model(
        x, labels, np.ones(len(x), dtype=float)
    )
    assert all(value >= 0.0 for value in model["coefficients"])
    assert metrics["weighted_auc"] > 0.80
