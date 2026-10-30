from types import SimpleNamespace

from lawn_mower_console.rtk_formatter import format_rtk_log


def test_formats_rk3588_rtk_fields():
    message = SimpleNamespace(
        log_level="INFO",
        ins_status=4,
        gps_num_satellites=21,
        latitude=31.123456,
        longitude=121.654321,
        altitude=12.3,
        lat_std=1.09,
        lon_std=1.08,
        h_std=1.2,
        temperature=37.5,
        non_rtk_fixed_count=6,
        non_rtk_fixed_duration=2.5,
        error_type="",
        error_description="",
    )

    line = format_rtk_log(message, 1_000_000_000)

    assert line.level == "INFO"
    for text in ("INS:4", "卫星:21", "31.123456", "温度:37.5", "非固定:6/2.5s"):
        assert text in line.text


def test_formats_new_network_rtk_fields():
    message = SimpleNamespace(
        log_level="WARN",
        connection_phase="RUNNING",
        network_status="DELAY_WARNING",
        fix_quality=4,
        num_satellites=18,
        hdop=0.8,
        differential_age=1.2,
        latitude=30.0,
        longitude=120.0,
        altitude=8.0,
        error_type="",
        error_description="",
    )

    line = format_rtk_log(message, 2)

    assert line.level == "WARN"
    for text in ("RUNNING", "DELAY_WARNING", "FIX:4", "卫星:18", "HDOP:0.80", "差分:1.2s"):
        assert text in line.text


def test_error_fields_have_priority_and_unknown_level_normalizes():
    message = SimpleNamespace(
        log_level="fatal",
        error_type="SOCKET",
        error_description="connection lost",
        latitude=0.0,
        longitude=0.0,
    )

    line = format_rtk_log(message, 3)

    assert line.level == "ERROR"
    assert line.text == "SOCKET: connection lost"
