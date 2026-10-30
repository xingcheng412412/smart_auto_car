"""Run launcher with fake ROS executables; never touch CAN or serial devices."""
import os
from pathlib import Path
import pty
import shutil
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("mode", ["console_exit", "backend_exit", "stop_and_duplicate"])
def test_session_lifecycle(tmp_path, mode):
    assert (ROOT / 'one_click/start.sh').exists()
    workspace = tmp_path / 'workspace'
    shutil.copytree(ROOT / 'one_click', workspace / 'one_click')
    origin = workspace / 'src/mower_start/param/gnss_init_pose.yaml'
    origin.parent.mkdir(parents=True)
    lidar_config = workspace / 'src/lawn_mower_tracker/config/fairy_avoidance_driver.yaml'
    lidar_config.parent.mkdir(parents=True)
    lidar_config.write_text('{}\n')
    origin.write_text('lat: 31.0\nlon: 121.0\nh: 0.0\n')
    setup = tmp_path / 'setup.bash'
    setup.write_text(':\n')
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    console = tmp_path / 'prefix/lib/lawn_mower_console/lawn_mower_console'
    console.parent.mkdir(parents=True)
    console.write_text('#!/usr/bin/python3\nimport time\ntime.sleep(' + ('0.5' if mode == 'console_exit' else '30') + ')\n')
    console.chmod(0o755)
    for package, name in [('can_client', 'vehicle_core'), ('rtk_gps_driver_cpp', 'rtk_gps_node_cpp'),
                          ('lawn_mower_recorder', 'recorder_enhanced.py'), ('lawn_mower_tracker', 'lawn_mower_tracker_node'), ('rslidar_sdk', 'rslidar_sdk_node')]:
        executable = tmp_path / 'prefix/lib' / package / name
        executable.parent.mkdir(parents=True)
        executable.write_text('#!/bin/bash\nexit 0\n')
        executable.chmod(0o755)
    ros2 = bindir / 'ros2' 
    ros2.write_text(f'''#!/bin/bash
if [[ "$1" == pkg ]]; then echo '{tmp_path}/prefix'; exit 0; fi
exec python3 -c 'import signal,time,os; from pathlib import Path; Path("{tmp_path}/backend.pid").write_text(str(os.getpid())); signal.signal(signal.SIGINT, lambda *_: exit(0)); time.sleep({0.2 if mode == "backend_exit" else 30})'
''')
    ros2.chmod(0o755)
    env = dict(os.environ, PATH=f'{bindir}:'+os.environ['PATH'],
               ROS_SETUP=str(setup), WORKSPACE_SETUP=str(setup), INIT_CAN='false')
    master, slave = pty.openpty()
    try:
        start = ['bash', './one_click/start.sh']
        process = subprocess.Popen(start, env=env, cwd=workspace, stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if mode == 'stop_and_duplicate':
            for _ in range(100):
                if (tmp_path / 'backend.pid').exists():
                    break
                time.sleep(0.02)
            duplicate = subprocess.run(start, env=env, cwd=workspace, stdin=slave, capture_output=True, timeout=5)
            assert duplicate.returncode == 1
            assert '已在运行' in duplicate.stderr.decode()
            stopped = subprocess.run(['bash', str(workspace / 'one_click/stop.sh')],
                                     capture_output=True, timeout=17)
            assert stopped.returncode == 0, stopped.stderr.decode()
        out, err = process.communicate(timeout=17)
    finally:
        os.close(master)
        os.close(slave)
    assert process.returncode == {'console_exit': 0, 'backend_exit': 1, 'stop_and_duplicate': 143}[mode], err.decode()
    backend_pid = int((tmp_path / 'backend.pid').read_text())
    assert not Path(f'/proc/{backend_pid}').exists()
    assert not (workspace / 'one_click/.run/session.pid').exists()
