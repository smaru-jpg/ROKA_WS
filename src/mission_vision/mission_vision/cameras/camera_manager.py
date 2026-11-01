
from mission_vision.cameras.ros_image_camera import RosImageCamera


class CameraManager:
    """Selects a camera backend from YAML configuration."""

    def __init__(self, config, node):
        camera_type = str(config.get("type", "ros")).lower()
        if camera_type != "ros":
            raise ValueError(
                "ROKA runtime requires camera.type: ros so one ROS 2 "
                "RealSense process can be shared by every mission"
            )
        self.camera = RosImageCamera(config, node)

        self.camera_type = camera_type

    def read(self):
        return self.camera.read()

    def age_seconds(self):
        return self.camera.age_seconds()

    def close(self):
        self.camera.close()
