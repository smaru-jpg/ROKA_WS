import time

from . import arm_config as cfg


class GraspSequence:
    def __init__(self, driver):
        self.driver = driver

    def make_grasp_poses(self, ready_pose):
        if (
            cfg.GRASP_OFFSET_11 is None
            or cfg.GRASP_OFFSET_13 is None
            or cfg.GRASP_OFFSET_WRIST is None
        ):
            return None

        set1 = {
            11: ready_pose[11] + cfg.GRASP_OFFSET_11,
            12: ready_pose[12] + cfg.GRASP_OFFSET_12,
            13: ready_pose[13] + cfg.GRASP_OFFSET_13,
            "wrist": ready_pose["wrist"] + cfg.GRASP_OFFSET_WRIST,
            "gripper": cfg.GRIPPER_OPEN_POSITION,
        }

        grip = dict(set1)
        grip["gripper"] = cfg.GRIPPER_CLOSE_POSITION

        return {
            "SET1": set1,
            "GRIP": grip,
            "HOME2": dict(cfg.HOME2_POSE),
            "PLACE": dict(cfg.PLACE_POSE),
            "DROP": dict(cfg.DROP_POSE),
            "SECTION2_DRIVE": dict(cfg.SECTION2_DRIVE_POSE),
        }

    def run(self, poses):
        # SET1
        if not self.driver.move_pose_values(
            "SET1",
            poses["SET1"],
        ):
            return False

        if not self.driver.wait_until_pose_values_reached(
            "SET1",
            poses["SET1"],
        ):
            return False

        # GRIP
        if not self.driver.move_pose_values(
            "GRIP",
            poses["GRIP"],
        ):
            return False

        if not self.driver.wait_until_pose_values_reached(
            "GRIP",
            poses["GRIP"],
        ):
            return False

        print(f"[GRIP] 파지력 {cfg.GRIP_HOLD_TIME:.1f}초 유지")
        time.sleep(cfg.GRIP_HOLD_TIME)

        # HOME2
        if not self.driver.move_pose_values(
            "HOME2",
            poses["HOME2"],
        ):
            return False

        if not self.driver.wait_until_pose_values_reached(
            "HOME2",
            poses["HOME2"],
        ):
            return False

        print()
        print("=" * 60)
        print("파지 + HOME2 복귀 완료")
        print("Gripper CLOSE Goal / Torque 유지 중")
        print("=" * 60)

        return True

    def run_post_grasp(self, poses):
        """Place the verified box, release it, and enter the drive pose."""
        for name in ("PLACE", "DROP", "SECTION2_DRIVE"):
            if not self.driver.move_pose_values(name, poses[name]):
                return False
            if not self.driver.wait_until_pose_values_reached(name, poses[name]):
                return False

        print()
        print("=" * 60)
        print("박스 배치 + SECTION2_DRIVE 자세 전환 완료")
        print("Dynamixel Torque 유지 중")
        print("=" * 60)
        return True
