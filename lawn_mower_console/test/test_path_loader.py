import math

import pytest

from lawn_mower_console.path_loader import (
    load_target_path,
    validate_map_name,
)


@pytest.mark.parametrize("name", ["", ".", "..", "../x", "a/b", "/tmp/x"])
def test_map_name_rejects_unsafe_names(name):
    with pytest.raises(ValueError, match="one directory"):
        validate_map_name(name)


def test_loads_pose_xy_columns(tmp_path):
    map_dir = tmp_path / "map_01"
    map_dir.mkdir()
    (map_dir / "lawn_mower_records.csv").write_text(
        "id,pose_x,pose_y\n"
        "1,1.5,2.5\n"
        "2,3.0,4.0\n"
        "3,5.0,6.0\n"
    )

    points = load_target_path(
        tmp_path, "map_01", "lawn_mower_records.csv"
    )

    assert [(point.x, point.y) for point in points] == [
        (1.5, 2.5),
        (3.0, 4.0),
        (5.0, 6.0),
    ]


def test_missing_path_file_has_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="path file not found"):
        load_target_path(tmp_path, "map_01", "lawn_mower_records.csv")


def test_missing_xy_columns_is_rejected(tmp_path):
    map_dir = tmp_path / "map_01"
    map_dir.mkdir()
    (map_dir / "path.csv").write_text("x,y\n1,2\n3,4\n5,6\n")

    with pytest.raises(ValueError, match="pose_x and pose_y"):
        load_target_path(tmp_path, "map_01", "path.csv")


@pytest.mark.parametrize("bad_value", ["nan", "inf", "-inf", "text"])
def test_non_finite_or_invalid_values_are_rejected(tmp_path, bad_value):
    map_dir = tmp_path / "map_01"
    map_dir.mkdir()
    (map_dir / "path.csv").write_text(
        "pose_x,pose_y\n"
        "0,0\n"
        f"{bad_value},1\n"
        "2,2\n"
    )

    with pytest.raises(ValueError, match="finite numbers"):
        load_target_path(tmp_path, "map_01", "path.csv")


def test_path_requires_at_least_three_points(tmp_path):
    map_dir = tmp_path / "map_01"
    map_dir.mkdir()
    (map_dir / "path.csv").write_text(
        "pose_x,pose_y\n0,0\n1,1\n"
    )

    with pytest.raises(ValueError, match="at least three"):
        load_target_path(tmp_path, "map_01", "path.csv")
