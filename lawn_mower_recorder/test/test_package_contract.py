from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_package_is_ros2_ament_with_required_dependencies():
    root = ET.parse(ROOT / "package.xml").getroot()
    assert root.attrib["format"] == "3"
    assert root.findtext("export/build_type") == "ament_cmake"
    dependencies = {node.text for node in root if node.tag.endswith("depend")}
    assert {
        "ament_cmake",
        "rclpy",
        "geometry_msgs",
        "sensor_msgs",
        "rosidl_default_generators",
        "rosidl_default_runtime",
    } <= dependencies
    assert "tf_transformations" not in dependencies


def test_cmake_generates_interfaces_and_installs_node():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "find_package(ament_cmake_python REQUIRED)" in cmake
    assert "ament_get_python_install_dir(PYTHON_INSTALL_DIR)" in cmake
    assert "DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}" in cmake
    assert "ament_python_install_package" not in cmake
    assert "rosidl_generate_interfaces" in cmake
    assert '"msg/RecordStatus.msg"' in cmake
    assert '"srv/ProcessSwitch.srv"' in cmake
    assert '"srv/RecordMarkPoint.srv"' in cmake
    assert "lawn_mower_recorder/recorder_enhanced.py" in cmake
    assert "ament_package()" in cmake


def test_cmake_disables_incompatible_external_pytest_plugins():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert 'set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")' in cmake
    assert 'set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "${_PYTEST_SAVED}")' in cmake


def test_core_test_prepends_source_python_path():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert (
        '"PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"'
        in cmake
    )


def test_ros1_and_can_apis_are_absent():
    sources = "\n".join(
        path.read_text()
        for path in ROOT.rglob("*")
        if (
            path.is_file()
            and path.suffix in {".py", ".xml", ".txt"}
            and "test" not in path.parts
        )
    )
    assert "rospy" not in sources
    assert "catkin" not in sources
    assert "VehicleStatus" not in sources
    assert "knife_state" not in sources


def test_node_uses_rclpy_and_declares_runtime_parameters():
    node = (ROOT / "lawn_mower_recorder/recorder_enhanced.py").read_text()
    assert "class LawnMowerRecorder(Node):" in node
    assert "create_subscription(" in node
    assert "PoseStamped," in node
    assert "NavSatFix," in node
    assert "create_service(" in node
    assert "ProcessSwitch," in node
    assert "RecordMarkPoint," in node
    for name in (
        "base_dir",
        "pose_topic",
        "fix_topic",
        "record_status_topic",
        "record_period",
        "status_period",
    ):
        assert re.search(rf'declare_parameter\(\s*"{name}"', node)
    assert "VehicleFeedback" not in node
    assert "time.sleep" not in node


def test_recorder_uses_workspace_recorded_files_default():
    expected = "~/fw_mini_pro_control_ros2/recorded_files"
    node = (ROOT / "lawn_mower_recorder/recorder_enhanced.py").read_text()
    launch = (ROOT / "launch/data_record.launch.py").read_text()
    assert expected in node
    assert expected in launch
    assert "Five_blade_lawn_mower/recorded_files" not in node + launch
