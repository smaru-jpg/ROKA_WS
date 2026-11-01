from roka_mission import section5_lifecycle_gate as gate_module
from roka_mission.section5_lifecycle_gate import (
    Section5LifecycleGate,
    state_request_expired,
)


def test_lifecycle_state_request_expires_at_configured_timeout():
    assert not state_request_expired(10.0, 10.99, 1.0)
    assert state_request_expired(10.0, 11.0, 1.0)
    assert state_request_expired(10.0, 12.0, 1.0)


def test_expired_lifecycle_request_is_cancelled_and_retried(monkeypatch):
    class Future:
        def __init__(self):
            self.cancelled = False
            self.callback = None

        def cancel(self):
            self.cancelled = True

        def add_done_callback(self, callback):
            self.callback = callback

    class Client:
        def __init__(self):
            self.new_future = Future()
            self.calls = 0

        def service_is_ready(self):
            return True

        def call_async(self, _request):
            self.calls += 1
            return self.new_future

    class Logger:
        def warning(self, _message):
            pass

    old_future = Future()
    client = Client()
    gate = Section5LifecycleGate.__new__(Section5LifecycleGate)
    gate.state_clients = {"safety": client}
    gate.pending = {"safety": (old_future, 10.0)}
    gate.states = {"safety": True}
    gate.state_request_timeout = 1.0
    gate._zero = lambda: None
    gate.get_logger = lambda: Logger()
    gate._state_result = lambda _name, _future: None
    gate._ready = lambda: all(gate.states.values())
    monkeypatch.setattr(gate_module.time, "monotonic", lambda: 12.0)

    Section5LifecycleGate._check_states(gate)

    assert old_future.cancelled
    assert client.calls == 1
    assert gate.pending["safety"] == (client.new_future, 12.0)
