from types import SimpleNamespace

from roka_arm import arm_control_node as control_module
from roka_arm.arm_control_node import ArmControlNode


class Logger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)

    def warning(self, _message):
        pass


def test_home2_and_existing_current_check_precede_new_place_sequence(monkeypatch):
    events = []

    class Grasp:
        def run(self, _poses):
            events.append("SET1_GRIP_HOME2")
            return True

        def run_post_grasp(self, _poses):
            events.append("PLACE_DROP_SECTION2_DRIVE")
            return True

    class Driver:
        def read_gripper_current_mA(self):
            events.append("VERIFY_CURRENT")
            return 350.0

    monkeypatch.setattr(control_module.time, "sleep", lambda _seconds: None)
    node = object.__new__(ArmControlNode)
    node.grasp = Grasp()
    node.driver = Driver()
    node.grasp_attempt_count = 1
    node.grasp_sequence_finished = False
    node.grasp_final_failed = False
    node.get_logger = lambda: Logger()

    node._run_grasp({})

    assert node.grasp_sequence_finished
    assert not node.grasp_final_failed
    assert events[0] == "SET1_GRIP_HOME2"
    assert "VERIFY_CURRENT" in events[1:-1]
    assert events[-1] == "PLACE_DROP_SECTION2_DRIVE"
