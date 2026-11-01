from types import SimpleNamespace

import math

import pytest

from roka_mission.box_pick_coordinator import (
    BoxPickCoordinator,
    State,
    conservative_body_speed,
    observation_is_eligible,
)


def observation(**overrides):
    values = {
        "detected": True,
        "activation_roi": True,
        "depth_valid": True,
        "depth_mm": 800.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_image_position_does_not_gate_an_eligible_target():
    assert observation_is_eligible(observation(), 0.1, 0.4, 1800.0)
    assert observation_is_eligible(
        observation(activation_roi=False), 0.1, 0.4, 1800.0
    )


def test_stale_or_out_of_depth_target_is_rejected():
    assert not observation_is_eligible(observation(), 0.5, 0.4, 1800.0)
    assert not observation_is_eligible(
        observation(depth_mm=2000.0), 0.1, 0.4, 1800.0
    )


def test_any_fresh_body_source_reporting_motion_blocks_the_arm():
    samples = {
        "/lio/odom": (1_000_000_000, 0.0),
        "/odom": (1_100_000_000, 0.03),
    }
    assert conservative_body_speed(samples, 1_200_000_000, 0.5) == pytest.approx(
        0.03
    )


def test_missing_or_stale_body_odometry_is_not_treated_as_stopped():
    assert math.isinf(conservative_body_speed({}, 1_000_000_000, 0.5))
    samples = {"/lio/odom": (100_000_000, 0.0)}
    assert math.isinf(conservative_body_speed(samples, 1_000_000_000, 0.5))


def test_completed_arm_switches_box_to_traffic_before_releasing_hold():
    class Response:
        success = True
        message = "ok"

    class Future:
        def result(self):
            return Response()

    node = object.__new__(BoxPickCoordinator)
    node.pending = Future()
    transitions = []
    node._transition = lambda state, detail="": (
        setattr(node, "state", state), transitions.append(state)
    )

    node.state = State.DISABLE_ARM
    node._service_done(Future(), "disable arm")
    node._service_done(Future(), "disable box YOLO")
    node._service_done(Future(), "enable traffic YOLO")

    assert transitions == [
        State.DISABLE_BOX_VISION,
        State.ENABLE_TRAFFIC_VISION,
        State.RELEASE_HOLD,
    ]


def test_operator_trigger_requires_fresh_confirmed_box_and_locks_base():
    node = object.__new__(BoxPickCoordinator)
    node.operator_trigger_required = True
    node.required_frames = 4
    node.detection_frames = 3
    node.state = State.WAIT_TARGET
    response = SimpleNamespace(success=False, message="")
    node._operator_trigger(None, response)
    assert not response.success
    assert "BOX NOT READY" in response.message

    transitions = []
    node.detection_frames = 4
    node._transition = lambda state, detail="": (
        setattr(node, "state", state), transitions.append((state, detail))
    )
    node._operator_trigger(None, response)
    assert response.success
    assert transitions[-1][0] == State.STOP_ROUTE
    assert BoxPickCoordinator.state_requires_base_lock(State.STOP_ROUTE)
    assert not BoxPickCoordinator.state_requires_base_lock(State.COMPLETE)
