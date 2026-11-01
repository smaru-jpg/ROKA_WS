import time

from . import arm_config as cfg


class VisualServoController:
    """
    Apply the separated X/Y/Z visual-servo algorithm.

    기존 X/Y/Z Visual Servoing 알고리즘을 그대로 분리.
    실제 모터 명령은 DynamixelDriver를 통해 수행.
    """

    def __init__(self, driver):
        self.driver = driver

        self.last_x_command_time = 0.0
        self.last_y_command_time = 0.0
        self.last_z_command_time = 0.0

    def reset(self):
        self.last_x_command_time = 0.0
        self.last_y_command_time = 0.0
        self.last_z_command_time = 0.0

    def command_x_axis(self, cx):
        ex = cx - cfg.TARGET_X

        if abs(ex) <= cfg.DEAD_ZONE_X:
            return "X CENTER"

        now = time.monotonic()

        if now - self.last_x_command_time < cfg.X_COMMAND_INTERVAL:
            return "X WAIT"

        if ex > 0:
            delta = cfg.X_RIGHT_ERROR_DELTA_SIGN * cfg.X_DELTA
            direction_text = "BOX RIGHT"
        else:
            delta = -cfg.X_RIGHT_ERROR_DELTA_SIGN * cfg.X_DELTA
            direction_text = "BOX LEFT"

        print(
            f"[X SERVO] {direction_text} | "
            f"cx={cx}, ex={ex} | "
            f"ID {cfg.X_JOINT_ID} delta={delta}"
        )

        ok = self.driver.move_arm_delta(
            cfg.X_JOINT_ID,
            delta,
        )

        if ok:
            self.last_x_command_time = now
            return "X MOVE"

        return "X ERROR"

    def command_y_axis(self, cy):
        ey = cy - cfg.TARGET_Y

        if abs(ey) <= cfg.DEAD_ZONE_Y:
            return "Y CENTER"

        now = time.monotonic()

        if now - self.last_y_command_time < cfg.Y_COMMAND_INTERVAL:
            return "Y WAIT"

        if ey > 0:
            delta = cfg.Y_DOWN_ERROR_DELTA_SIGN * cfg.Y_DELTA
            direction_text = "BOX DOWN"
        else:
            delta = -cfg.Y_DOWN_ERROR_DELTA_SIGN * cfg.Y_DELTA
            direction_text = "BOX UP"

        print(
            f"[Y SERVO] {direction_text} | "
            f"cy={cy}, ey={ey} | "
            f"Wrist ID4 delta={delta}"
        )

        ok = self.driver.move_wrist_delta(delta)

        if ok:
            self.last_y_command_time = now
            return "Y MOVE"

        return "Y ERROR"

    def command_z_approach(self):
        now = time.monotonic()

        if now - self.last_z_command_time < cfg.Z_COMMAND_INTERVAL:
            return "Z WAIT"

        present_11 = self.driver.read_h54_position(cfg.JOINT_1_ID)

        if present_11 is None:
            return "Z ERROR"

        # ID11 우선 -> 0 근처
        if present_11 < -cfg.JOINT_11_ZERO_TOLERANCE:
            goal_11 = present_11 + abs(cfg.Z11_DELTA)

            if goal_11 > 0:
                goal_11 = 0

            delta_11 = goal_11 - present_11

            print(
                f"[Z APPROACH-11] "
                f"ID 11: {present_11} -> {goal_11} "
                f"(delta={delta_11})"
            )

            ok = self.driver.write_arm_position(
                cfg.JOINT_1_ID,
                goal_11,
            )

            if ok:
                self.last_z_command_time = now
                return "Z ID11 MOVE"

            return "Z ERROR"

        # ID11 = 0±tolerance -> ID13
        print(
            f"[Z SWITCH] ID 11={present_11} "
            f"(0±{cfg.JOINT_11_ZERO_TOLERANCE} 도달) -> "
            f"ID 13 사용 | ID 13 delta={cfg.Z13_DELTA}"
        )

        ok = self.driver.move_arm_delta(
            cfg.JOINT_3_ID,
            cfg.Z13_DELTA,
        )

        if ok:
            self.last_z_command_time = now
            return "Z ID13 MOVE"

        return "Z ERROR"
