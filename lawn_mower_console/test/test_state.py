from lawn_mower_console.rtk_formatter import RtkLogLine
from lawn_mower_console.state import ConsoleState


def test_rtk_logs_are_bounded_and_clearable():
    state = ConsoleState(max_rtk_log_lines=3)
    for index in range(5):
        state.append_rtk_log(
            RtkLogLine(index, "INFO", f"log {index}")
        )

    assert state.snapshot(5).rtk_logs == (
        RtkLogLine(2, "INFO", "log 2"),
        RtkLogLine(3, "INFO", "log 3"),
        RtkLogLine(4, "INFO", "log 4"),
    )
    state.clear_rtk_logs()
    assert state.snapshot(5).rtk_logs == ()


def test_task_mutual_exclusion():
    state = ConsoleState()
    state.set_record_status(1, 10, 2.0, "map_a", 1)
    assert state.can_start_tracking() == (False, "recording is active")

    state.set_record_status(0, 10, 2.0, "", 2)
    state.set_tracker_status(3, 1, 3)
    assert state.can_start_recording() == (False, "tracking is active")

    state.set_tracker_status(4, 1, 4)
    assert state.can_start_recording() == (False, "tracking is active")


def test_pending_operation_suppresses_duplicates():
    state = ConsoleState()
    assert state.begin_operation("start recording")
    assert not state.begin_operation("start tracking")
    state.finish_operation("start recording", True, "started")
    assert state.begin_operation("start tracking")


def test_snapshot_reports_gnss_freshness():
    state = ConsoleState(gnss_timeout_sec=1.0)
    state.update_pose(1.0, 2.0, 3.0, 0.5, 1_000_000_000)

    snapshot = state.snapshot(1_500_000_000)
    assert snapshot.gnss_fresh
    assert (snapshot.pose_x, snapshot.pose_y, snapshot.pose_z) == (1.0, 2.0, 3.0)
    assert not state.snapshot(2_100_000_000).gnss_fresh


def test_snapshot_collections_are_immutable_copies():
    state = ConsoleState()
    state.append_rtk_log(RtkLogLine(1, "INFO", "one"))
    snapshot = state.snapshot(1)
    state.append_rtk_log(RtkLogLine(2, "INFO", "two"))
    assert snapshot.rtk_logs == (RtkLogLine(1, "INFO", "one"),)
