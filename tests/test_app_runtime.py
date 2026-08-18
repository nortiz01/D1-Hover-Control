from __future__ import annotations

import copy
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

import d1_hover_control_app as app


def new_state() -> app.SharedState:
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    app.normalize_settings(settings)
    return app.SharedState(settings)


def test_default_kinematic_chain_is_parsed_once(monkeypatch):
    app.default_movable_chain.cache_clear()
    original_load = app.load_joints
    calls = 0

    def counted_load(path):
        nonlocal calls
        calls += 1
        return original_load(path)

    monkeypatch.setattr(app, "load_joints", counted_load)
    try:
        first = app.default_movable_chain()
        second = app.default_movable_chain()
    finally:
        app.default_movable_chain.cache_clear()

    assert first is second
    assert len(first) == 6
    assert calls == 1


def test_bridge_status_external_process_check_is_ttl_cached(monkeypatch):
    state = new_state()
    calls = 0

    def fake_run(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        return SimpleNamespace(stdout="123 multiple_joint_angle_control --port 8888\n")

    monkeypatch.setattr(app.subprocess, "run", fake_run)

    assert app.bridge_status(state)["running"] is True
    assert app.bridge_status(state)["running"] is True
    assert calls == 1

    with state.lock:
        state.bridge_status_cache_time -= app.BRIDGE_STATUS_TTL_SEC + 0.1
    assert app.bridge_status(state)["running"] is True
    assert calls == 2


def test_rendered_frame_is_cached_until_source_or_settings_change(monkeypatch):
    state = new_state()
    ok, encoded = cv2.imencode(".jpg", np.zeros((24, 32, 3), dtype=np.uint8))
    assert ok
    state.update_frame(encoded.tobytes(), {"tag_found": False}, ("127.0.0.1", 4321))

    overlay_calls = 0

    def fake_overlay(frame, _snapshot):
        nonlocal overlay_calls
        overlay_calls += 1
        return frame

    monkeypatch.setattr(app, "draw_overlay", fake_overlay)

    first = app.render_latest_jpeg(state)
    second = app.render_latest_jpeg(state)
    assert first is second
    assert overlay_calls == 1

    with state.lock:
        state.settings["hover_z_m"] += 0.01
        state.mark_settings_changed()
    third = app.render_latest_jpeg(state)
    assert third
    assert overlay_calls == 2


def test_manual_and_follow_solves_cannot_overlap(monkeypatch):
    state = new_state()
    activity_lock = threading.Lock()
    active = 0
    maximum_active = 0
    barrier = threading.Barrier(3)
    results = []

    def fake_solve(_state, dry_run, source):
        nonlocal active, maximum_active
        with activity_lock:
            active += 1
            maximum_active = max(maximum_active, active)
        time.sleep(0.04)
        with activity_lock:
            active -= 1
        return {"source": source, "dry_run": dry_run}

    monkeypatch.setattr(app, "_solve_hover_target", fake_solve)

    def invoke(source):
        barrier.wait()
        results.append(app.solve_hover_target(state, dry_run=False, source=source))

    threads = [
        threading.Thread(target=invoke, args=("manual",)),
        threading.Thread(target=invoke, args=("follow",)),
    ]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert maximum_active == 1
    assert {result["source"] for result in results} == {"manual", "follow"}


def test_bridge_launch_uses_configured_udp_port_and_closes_log(tmp_path: Path, monkeypatch):
    state = new_state()
    state.settings["udp_port"] = 9123
    log_dir = tmp_path / "logs"
    build_dir = tmp_path / "d1_sdk" / "build_project430"
    build_dir.mkdir(parents=True)
    captured = {}

    class FakeProcess:
        pid = 2468

        def __init__(self):
            self.running = True

        def poll(self):
            return None if self.running else 0

        def terminate(self):
            self.running = False

        def wait(self, timeout=None):
            return 0

    process = FakeProcess()

    def fake_popen(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return process

    monkeypatch.setattr(app, "ROOT", tmp_path)
    monkeypatch.setattr(app, "LOG_DIR", log_dir)
    monkeypatch.setattr(app.subprocess, "Popen", fake_popen)

    started = app.start_bridge(state)
    assert started["port"] == 9123
    assert captured["command"][-2:] == ["--port", "9123"]
    assert state.bridge_log_handle is not None

    stopped = app.stop_bridge(state)
    assert stopped["ok"] is True
    assert state.bridge_log_handle is None
    assert process.running is False
