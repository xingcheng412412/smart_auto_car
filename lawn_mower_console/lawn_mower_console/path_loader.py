import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class Point:
    x: float
    y: float


def validate_map_name(name):
    candidate = Path(name)
    if (
        not name
        or candidate.is_absolute()
        or len(candidate.parts) != 1
        or name in {".", ".."}
    ):
        raise ValueError("map name must be one directory name")
    return name


def load_target_path(root, map_name, filename):
    safe_map_name = validate_map_name(map_name)
    if not filename or Path(filename).name != filename:
        raise ValueError("path filename must be one file name")
    path = Path(root).expanduser() / safe_map_name / filename
    if not path.is_file():
        raise FileNotFoundError(f"path file not found: {path}")

    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        if not {"pose_x", "pose_y"} <= fields:
            raise ValueError("path CSV must contain pose_x and pose_y")

        points = []
        for row_number, row in enumerate(reader, start=2):
            try:
                x = float(row["pose_x"])
                y = float(row["pose_y"])
            except (TypeError, ValueError):
                raise ValueError(
                    f"path row {row_number} must contain finite numbers"
                ) from None
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(
                    f"path row {row_number} must contain finite numbers"
                )
            points.append(Point(x, y))

    if len(points) < 3:
        raise ValueError("path must contain at least three points")
    return points
