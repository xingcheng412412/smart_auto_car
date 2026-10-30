from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPECTED = "~/fw_mini_pro_control_ros2/recorded_files"


def test_tracker_config_and_node_use_workspace_recorded_files_default():
    config = (ROOT / "config/tracker.yaml").read_text()
    node = (ROOT / "src/tracker_node.cpp").read_text()
    assert EXPECTED in config
    assert EXPECTED in node
    assert "Five_blade_lawn_mower/recorded_files" not in config + node
