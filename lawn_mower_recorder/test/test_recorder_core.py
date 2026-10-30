import csv
import math

import pytest

from lawn_mower_recorder.recorder_core import (
    CSV_HEADER,
    MARK_HEADER,
    RecordingSession,
    quaternion_to_euler,
    validate_map_name,
)


def sample_row():
    return list(range(len(CSV_HEADER)))


def test_headers_remove_knife_state_and_mark_adds_point_mark():
    assert "knife_state" not in CSV_HEADER
    assert MARK_HEADER == CSV_HEADER + ["point_mark"]


@pytest.mark.parametrize("name", [".", "..", "../escape", "a/b", "/tmp/x"])
def test_map_name_rejects_path_components(name):
    with pytest.raises(ValueError):
        validate_map_name(name)


def test_empty_map_name_means_do_not_move():
    assert validate_map_name("") is None


def test_session_writes_row_and_mark_then_moves_files(tmp_path):
    session = RecordingSession(tmp_path)
    session.start()
    session.append(sample_row())
    session.mark(7)
    destination = session.stop("map_01")

    assert destination == tmp_path / "map_01"
    with (destination / "lawn_mower_records.csv").open(newline="") as stream:
        assert list(csv.reader(stream)) == [
            CSV_HEADER,
            [str(value) for value in sample_row()],
        ]
    with (destination / "mark_points.csv").open(newline="") as stream:
        rows = list(csv.reader(stream))
    assert rows == [
        MARK_HEADER,
        [str(value) for value in sample_row()] + ["7"],
    ]


def test_stop_without_map_name_leaves_record_in_base_dir(tmp_path):
    session = RecordingSession(tmp_path)
    session.start()
    session.stop("")

    assert (tmp_path / "lawn_mower_records.csv").exists()


def test_mark_requires_active_session_and_recorded_row(tmp_path):
    session = RecordingSession(tmp_path)
    with pytest.raises(RuntimeError, match="not recording"):
        session.mark(1)

    session.start()
    with pytest.raises(RuntimeError, match="no recorded point"):
        session.mark(1)


def test_append_rejects_wrong_column_count(tmp_path):
    session = RecordingSession(tmp_path)
    session.start()

    with pytest.raises(ValueError, match="columns"):
        session.append([1])


def test_cleanup_is_idempotent(tmp_path):
    session = RecordingSession(tmp_path)
    session.start()
    session.cleanup()
    session.cleanup()

    assert not session.active


def test_quaternion_to_euler_returns_known_yaw():
    half_yaw = math.pi / 4.0
    roll, pitch, yaw = quaternion_to_euler(
        0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)
    )

    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(math.pi / 2.0)
