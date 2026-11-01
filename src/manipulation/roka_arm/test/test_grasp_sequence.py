from roka_arm import arm_config as cfg
from roka_arm.grasp_sequence import GraspSequence


class RecordingDriver:
    def __init__(self):
        self.calls = []

    def move_pose_values(self, name, pose):
        self.calls.append(("move", name, dict(pose)))
        return True

    def wait_until_pose_values_reached(self, name, pose):
        self.calls.append(("wait", name, dict(pose)))
        return True


def test_existing_grasp_and_home2_order_is_preserved_before_verification():
    driver = RecordingDriver()
    sequence = GraspSequence(driver)
    ready = {11: 0, 12: 0, 13: 125480, "wrist": 1100, "gripper": 1400}
    poses = sequence.make_grasp_poses(ready)

    assert sequence.run(poses)
    assert [name for operation, name, _ in driver.calls if operation == "move"] == [
        "SET1",
        "GRIP",
        "HOME2",
    ]


def test_verified_grasp_runs_place_drop_then_section2_drive():
    driver = RecordingDriver()
    sequence = GraspSequence(driver)
    ready = {11: 0, 12: 0, 13: 125480, "wrist": 1100, "gripper": 1400}
    poses = sequence.make_grasp_poses(ready)

    assert poses["PLACE"] == cfg.PLACE_POSE
    assert poses["DROP"] == cfg.DROP_POSE
    assert poses["SECTION2_DRIVE"] == cfg.SECTION2_DRIVE_POSE
    assert sequence.run_post_grasp(poses)
    assert [name for operation, name, _ in driver.calls if operation == "move"] == [
        "PLACE",
        "DROP",
        "SECTION2_DRIVE",
    ]
