from dataclasses import dataclass


@dataclass(frozen=True)
class RtkLogLine:
    received_ns: int
    level: str
    text: str


def _optional(message, name):
    value = getattr(message, name, None)
    return value if value not in (None, "") else None


def _level(message):
    value = str(getattr(message, "log_level", "INFO")).upper()
    return value if value in {"INFO", "WARN", "ERROR"} else "ERROR"


def format_rtk_log(message, received_ns):
    level = _level(message)
    error_type = _optional(message, "error_type")
    error_description = _optional(message, "error_description")
    if error_type or error_description:
        if error_type and error_description:
            text = f"{error_type}: {error_description}"
        else:
            text = str(error_type or error_description)
        return RtkLogLine(int(received_ns), "ERROR", text)

    fields = []
    connection_phase = _optional(message, "connection_phase")
    network_status = _optional(message, "network_status")
    if connection_phase:
        fields.append(str(connection_phase))
    if network_status:
        fields.append(str(network_status))

    ins_status = _optional(message, "ins_status")
    fix_quality = _optional(message, "fix_quality")
    if ins_status is not None:
        fields.append(f"INS:{ins_status}")
    if fix_quality is not None:
        fields.append(f"FIX:{fix_quality}")

    satellites = _optional(message, "gps_num_satellites")
    if satellites is None:
        satellites = _optional(message, "num_satellites")
    if satellites is not None:
        fields.append(f"卫星:{satellites}")

    latitude = _optional(message, "latitude")
    longitude = _optional(message, "longitude")
    altitude = _optional(message, "altitude")
    if latitude is not None and longitude is not None:
        location = f"LL:{float(latitude):.6f},{float(longitude):.6f}"
        if altitude is not None:
            location += f",{float(altitude):.1f}m"
        fields.append(location)

    lat_std = _optional(message, "lat_std")
    lon_std = _optional(message, "lon_std")
    h_std = _optional(message, "h_std")
    if lat_std is not None and lon_std is not None:
        deviation = f"STD:{float(lat_std):.2f}/{float(lon_std):.2f}"
        if h_std is not None:
            deviation += f"/{float(h_std):.2f}"
        fields.append(deviation)

    hdop = _optional(message, "hdop")
    if hdop is not None:
        fields.append(f"HDOP:{float(hdop):.2f}")
    differential_age = _optional(message, "differential_age")
    if differential_age is not None:
        fields.append(f"差分:{float(differential_age):.1f}s")

    non_fixed_count = _optional(message, "non_rtk_fixed_count")
    non_fixed_duration = _optional(message, "non_rtk_fixed_duration")
    if non_fixed_count is not None:
        text = f"非固定:{non_fixed_count}"
        if non_fixed_duration is not None:
            text += f"/{float(non_fixed_duration):.1f}s"
        fields.append(text)

    temperature = _optional(message, "temperature")
    if temperature is not None:
        fields.append(f"温度:{float(temperature):.1f}C")

    return RtkLogLine(
        int(received_ns), level, " ".join(fields) or "RTK状态更新"
    )
