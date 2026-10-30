import csv
import math
from pathlib import Path
import threading


CSV_HEADER = [
    "id",
    "path_id",
    "timestamp",
    "pose_x",
    "pose_y",
    "pose_z",
    "yaw",
    "pitch",
    "roll",
    "linear_velocity",
    "linear_velocity_direction",
    "angular_velocity",
    "segment_type",
    "distance_from_prev",
    "cumulative_distance",
    "latitude",
    "longitude",
]
MARK_HEADER = CSV_HEADER + ["point_mark"]


def quaternion_to_euler(x, y, z, w):
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_sine = 2.0 * (w * y - z * x)
    if abs(pitch_sine) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, pitch_sine)
    else:
        pitch = math.asin(pitch_sine)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def validate_map_name(name):
    if not name:
        return None
    candidate = Path(name)
    if (
        candidate.is_absolute()
        or len(candidate.parts) != 1
        or name in {".", ".."}
    ):
        raise ValueError("map_name must be one directory name")
    return name


class RecordingSession:
    def __init__(self, base_dir):
        self.base_dir = Path(base_dir)
        self.record_path = self.base_dir / "lawn_mower_records.csv"
        self.mark_path = self.base_dir / "mark_points.csv"
        self.active = False
        self.last_row = None
        self._file = None
        self._writer = None
        self._lock = threading.Lock()

    def start(self):
        self.cleanup()
        self.base_dir.mkdir(parents=True, exist_ok=True)
        if self.mark_path.exists():
            self.mark_path.unlink()
        self._file = self.record_path.open("w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)
        self._file.flush()
        self.last_row = None
        self.active = True
        return self.record_path

    def append(self, row):
        if not self.active or self._writer is None:
            raise RuntimeError("not recording")
        if len(row) != len(CSV_HEADER):
            raise ValueError(
                f"record row has {len(row)} columns; expected {len(CSV_HEADER)}"
            )
        row_snapshot = list(row)
        with self._lock:
            self._writer.writerow(row_snapshot)
            self._file.flush()
            self.last_row = row_snapshot

    def mark(self, mark_id):
        if not self.active:
            raise RuntimeError("not recording")
        with self._lock:
            if self.last_row is None:
                raise RuntimeError("no recorded point")
            row_snapshot = list(self.last_row)
            new_file = not self.mark_path.exists()
            with self.mark_path.open("a", newline="") as stream:
                writer = csv.writer(stream)
                if new_file:
                    writer.writerow(MARK_HEADER)
                writer.writerow(row_snapshot + [mark_id])
        return self.mark_path

    def stop(self, map_name):
        safe_name = validate_map_name(map_name)
        self.cleanup()
        if safe_name is None:
            return None

        destination = self.base_dir / safe_name
        destination.mkdir(parents=True, exist_ok=True)
        if self.record_path.exists():
            self.record_path.replace(destination / self.record_path.name)
        if self.mark_path.exists():
            self.mark_path.replace(destination / self.mark_path.name)
        return destination

    def cleanup(self):
        with self._lock:
            if self._file is not None:
                self._file.close()
            self._file = None
            self._writer = None
            self.active = False
