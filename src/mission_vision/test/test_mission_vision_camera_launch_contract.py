from pathlib import Path


def test_general_mission_vision_is_color_only_at_30_fps():
    package = Path(__file__).resolve().parents[1]
    launch_source = (
        package / "mission_vision" / "launch_support.py"
    ).read_text(encoding="utf-8")

    assert '"rgb_camera.color_profile": "640,480,30"' in launch_source
    assert '"enable_depth": "false"' in launch_source
    assert '"enable_sync": "false"' in launch_source
    assert '"align_depth.enable": "false"' in launch_source
    assert "depth_module.depth_profile" not in launch_source
    assert 'DeclareLaunchArgument("enabled", default_value="true")' in launch_source
    assert 'LaunchConfiguration("enabled")' in launch_source


def test_sections_without_yolo_have_a_color_camera_view():
    package = Path(__file__).resolve().parents[1]
    launch_source = (package / "launch" / "camera_view.launch.py").read_text(
        encoding="utf-8"
    )
    assert "realsense_group" in launch_source
    assert 'executable="camera_view_node"' in launch_source
    assert 'DeclareLaunchArgument("display_enabled", default_value="true")' in launch_source
