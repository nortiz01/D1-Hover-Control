from __future__ import annotations

import copy
from pathlib import Path

import pytest

import d1_hover_control_app as app


def valid_settings() -> dict:
    return copy.deepcopy(app.DEFAULT_SETTINGS)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("ik_tol_m", float("nan")),
        ("ik_damping", float("inf")),
        ("udp_port", 0),
        ("udp_port", 65536),
        ("udp_port", True),
        ("udp_repeat", 0),
        ("ik_max_iter", 1.5),
        ("stream_fps", 0),
        ("stream_jpeg_quality", 101),
        ("orientation_tol_deg", 0),
        ("orientation_weight", -0.01),
        ("gripper_angle", -0.01),
        ("gripper_open_angle", 90.01),
        ("gripper_closed_angle", -1),
        ("tag_green_align_gripper_angle", 91),
        ("force_send", "sometimes"),
        ("udp_host", "  "),
        ("go2_stream_script", ""),
    ],
)
def test_normalize_settings_rejects_invalid_values(key, value):
    settings = valid_settings()
    settings[key] = value

    with pytest.raises(app.SettingsValidationError):
        app.normalize_settings(settings)


def test_normalize_settings_rejects_unknown_keys_and_bad_tool_offset():
    settings = valid_settings()
    settings["surprise"] = "unused"
    with pytest.raises(app.SettingsValidationError, match="Unknown setting"):
        app.normalize_settings(settings)

    settings = valid_settings()
    settings["tool_offset_m"] = [0.1, float("nan"), 0.3]
    with pytest.raises(app.SettingsValidationError, match="finite"):
        app.normalize_settings(settings)


def test_save_and_load_settings_round_trip_atomically(tmp_path: Path):
    path = tmp_path / "settings.json"
    settings = valid_settings()
    settings["udp_port"] = "9000"
    settings["hover_z_m"] = "0.04"

    app.save_settings(settings, path)

    assert app.load_settings(path) == settings
    assert settings["udp_port"] == 9000
    assert settings["hover_z_m"] == 0.04
    assert list(tmp_path.glob(".settings.json.*.tmp")) == []


def test_failed_replace_preserves_existing_settings(tmp_path: Path, monkeypatch):
    path = tmp_path / "settings.json"
    original = b'{"original":true}\n'
    path.write_bytes(original)

    def fail_replace(_source, _destination):
        raise OSError("simulated replace failure")

    monkeypatch.setattr(app.os, "replace", fail_replace)
    with pytest.raises(OSError, match="simulated"):
        app.save_settings(valid_settings(), path)

    assert path.read_bytes() == original
    assert list(tmp_path.glob(".settings.json.*.tmp")) == []


@pytest.mark.parametrize("content", ["{", "[]", '"not an object"'])
def test_load_settings_reports_corrupt_content_without_overwriting(tmp_path: Path, content: str):
    path = tmp_path / "settings.json"
    path.write_text(content, encoding="utf-8")

    with pytest.raises(app.SettingsValidationError):
        app.load_settings(path)

    assert path.read_text(encoding="utf-8") == content
