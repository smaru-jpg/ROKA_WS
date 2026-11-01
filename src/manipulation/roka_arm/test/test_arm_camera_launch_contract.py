from pathlib import Path


def test_section2_arm_camera_keeps_synchronized_rgbd_at_30_fps():
    package = Path(__file__).resolve().parents[1]
    launch_source = (
        package / "launch" / "roka_arm.launch.py"
    ).read_text(encoding="utf-8")

    assert '"rgb_camera.color_profile": "640,480,30"' in launch_source
    assert '"depth_module.depth_profile": "640,480,30"' in launch_source
    assert '"enable_depth": "true"' in launch_source
    assert '"enable_sync": "true"' in launch_source
    assert '"align_depth.enable": "true"' in launch_source
