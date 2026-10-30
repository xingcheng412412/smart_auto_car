from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]


def test_launch_exposes_base_dir_and_starts_recorder():
    launch = (ROOT / "launch/data_record.launch.py").read_text()
    assert re.search(r'DeclareLaunchArgument\(\s*"base_dir"', launch)
    assert 'package="lawn_mower_recorder"' in launch
    assert 'executable="recorder_enhanced.py"' in launch
    assert '"base_dir": LaunchConfiguration("base_dir")' in launch
