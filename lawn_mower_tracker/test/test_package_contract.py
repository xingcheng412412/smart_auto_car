from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_package_is_ros2_only():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    package = (ROOT / "package.xml").read_text()

    assert "ament_cmake" in cmake
    assert "rosidl_generate_interfaces" in cmake
    assert "catkin" not in cmake + package

    for obsolete in ("roscpp", "lidar_sam", "turfeye_vision", "autoware", "mqtt"):
        assert obsolete not in cmake + package


def test_service_contract_has_no_obstacle_field():
    service = (ROOT / "srv" / "MowerTrackControl.srv").read_text()

    assert "string track_state" in service
    assert "string map_name" in service
    assert "uint32 line_id" in service
    assert "bool success" in service
    assert "string message" in service
    assert "obstacle" not in service


def test_status_contract_preserves_task_correlation():
    status = (ROOT / "msg" / "PathStatus.msg").read_text()

    assert "int32 tracker_state" in status
    assert "uint32 line_id" in status


def test_path_direction_metadata_is_public_and_documented():
    path_type = (ROOT / "include/lawn_mower_tracker/types.hpp").read_text()
    loader = (
        ROOT / "include/lawn_mower_tracker/path_loader.hpp"
    ).read_text() + (ROOT / "src/path_loader.cpp").read_text()
    readme = (ROOT / "README.md").read_text()

    assert "int direction" in path_type
    assert "path_id" in path_type
    assert "linear_velocity" in loader
    assert "linear_velocity" in readme
    assert "path_id" in readme
    assert "0.01 m/s" in readme
    assert "重复点" in readme


def test_obsolete_integrations_are_removed():
    text = "\n".join(
        path.read_text(errors="ignore")
        for path in ROOT.rglob("*")
        if path.is_file()
        and path != Path(__file__)
        and path.suffix in {".cpp", ".hpp", ".h", ".xml", ".txt", ".yaml", ".py"}
    )
    for obsolete in (
        "<ros/ros.h>", "lidar_sam", "turfeye_vision", "lidar_semantic",
        "autoware_", "mqtt_msgs", "CollisionChecker", "knife_state",
        "mower_canbase", "deepal_canbase",
    ):
        assert obsolete not in text
