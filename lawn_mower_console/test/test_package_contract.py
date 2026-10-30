from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_package_is_ament_python_with_required_dependencies():
    package = ET.parse(ROOT / "package.xml").getroot()
    assert package.findtext("export/build_type") == "ament_python"
    dependencies = {item.text for item in package.findall("exec_depend")}
    assert {
        "can_client",
        "geometry_msgs",
        "lawn_mower_recorder",
        "lawn_mower_tracker",
        "rclpy",
        "rtk_gps_driver_cpp",
    } <= dependencies


def test_console_entry_point_is_installed():
    setup = (ROOT / "setup.py").read_text()
    assert "lawn_mower_console = lawn_mower_console.main:main" in setup


def test_setup_avoids_deprecated_tests_require_metadata():
    setup = (ROOT / "setup.py").read_text()
    assert "tests_require" not in setup


def test_console_never_uses_vehicle_command():
    source_dir = ROOT / "lawn_mower_console"
    sources = "\n".join(
        path.read_text() for path in source_dir.glob("*.py")
    )
    assert "VehicleCmd" not in sources
    assert "create_publisher" not in sources


def test_node_uses_expected_subscriptions_and_services():
    source = (ROOT / "lawn_mower_console/console_node.py").read_text()
    for token in (
        "PoseStamped",
        "RecordStatus",
        "PathStatus",
        "VehicleFeedback",
        "ProcessSwitch",
        "MowerTrackControl",
    ):
        assert token in source
    for parameter in ("path_root", "gnss_timeout_sec"):
        assert re.search(
            rf'declare_parameter\(\s*"{parameter}"', source
        )
    assert "VehicleCmd" not in source
    assert "create_publisher" not in source
    assert "call_async" in source


def test_tui_has_required_keys_and_does_not_stop_on_exit():
    tui = (ROOT / "lawn_mower_console/tui.py").read_text()
    for key in ("R", "E", "T", "P", "C", "S", "X", "Q"):
        assert f'"{key}"' in tui
    assert '"F"' not in tui
    assert "RTK 状态日志" in tui
    assert "GNSS Pose" in tui
    assert "clear_rtk_logs" in tui
    assert "renderer" not in tui
    main = (ROOT / "lawn_mower_console/main.py").read_text()
    assert "curses.wrapper" in main
    assert "rclpy.spin" in main
    assert "stop_tracking" not in main
    assert "stop_recording" not in main


def test_pytest_disables_incompatible_external_anyio_plugin():
    config = (ROOT / "pytest.ini").read_text()
    assert "addopts = -p no:anyio" in config


def test_console_uses_workspace_recorded_files_default():
    source = (ROOT / "lawn_mower_console/console_node.py").read_text()
    assert "~/fw_mini_pro_control_ros2/recorded_files" in source
    assert "Five_blade_lawn_mower/recorded_files" not in source


def test_console_subscribes_to_typed_rtk_log():
    source = (ROOT / "lawn_mower_console/console_node.py").read_text()
    assert "from rtk_gps_driver_cpp.msg import RTKGPSLog" in source
    assert 'declare_parameter("rtk_log_topic", "/rtk_log")' in source
    assert 'declare_parameter("max_rtk_log_lines", 500)' in source
    assert "format_rtk_log" in source
