from roka_mission.traffic_signal_route_gate import (
    SignalDecision,
    SignalDecisionLatch,
    parse_result_labels,
    requests_manual_takeover,
)


def test_result_parser_preserves_deployed_blue_red_class_contract():
    assert parse_result_labels("BLUE") == {"BLUE"}
    assert parse_result_labels("red, blue") == {"RED", "BLUE"}
    assert parse_result_labels("NONE") == set()


def test_red_stops_and_missing_detection_keeps_the_stop_latched():
    latch = SignalDecisionLatch(stop_confirmations=1, go_confirmations=2)
    assert latch.update("RED") == SignalDecision.STOP
    assert latch.stop_latched
    assert latch.update("NONE") == SignalDecision.NONE
    assert latch.stop_latched


def test_blue_requires_confirmation_and_only_resumes_after_red():
    latch = SignalDecisionLatch(stop_confirmations=1, go_confirmations=2)
    assert latch.update("BLUE") == SignalDecision.NONE
    assert latch.update("BLUE") == SignalDecision.NONE
    assert latch.update("RED") == SignalDecision.STOP
    assert latch.update("BLUE") == SignalDecision.NONE
    assert latch.stop_latched
    assert latch.update("BLUE") == SignalDecision.GO
    assert not latch.stop_latched


def test_red_has_priority_if_both_classes_are_reported():
    latch = SignalDecisionLatch(stop_confirmations=1, go_confirmations=1)
    assert latch.update("BLUE,RED") == SignalDecision.STOP
    assert latch.stop_latched


def test_b_stop_is_distinguished_from_the_startup_help_message():
    assert not requests_manual_takeover(
        "B emergency/manual takeover ready; Y/A owned by mission"
    )
    assert requests_manual_takeover(
        "B STOP: cancellation requested; manual drive ready"
    )
    assert requests_manual_takeover(
        "manual takeover active: LB + stick; hold LT for turbo"
    )
