from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np

import test_d1_550_ik as ik_cli
import test_tags3d_d435i_stream as producer


ROOT = Path(__file__).resolve().parents[1]


def test_test_ik_cli_refuses_unconverged_udp_send(monkeypatch):
    args = SimpleNamespace(
        target_xyz=np.array([9.0, 9.0, 9.0]),
        tool_offset=np.zeros(3),
        urdf=Path("unused.urdf"),
        base_link="base_link",
        end_link="Empty_Link6",
        orientation="none",
        orientation_tol_deg=5.0,
        orientation_weight=0.15,
        tol=0.001,
        max_iter=1,
        damping=0.03,
        seq=1,
        gripper_angle=0.0,
        send_udp="127.0.0.1:8888",
        force=False,
    )
    monkeypatch.setattr(ik_cli, "parse_args", lambda: args)
    monkeypatch.setattr(ik_cli, "load_joints", lambda _path: [object()])
    monkeypatch.setattr(ik_cli, "find_chain", lambda *_args: [SimpleNamespace(joint_type="revolute", name="j")])
    monkeypatch.setattr(
        ik_cli,
        "solve_ik",
        lambda *_args, **_kwargs: (
            np.zeros(6),
            np.zeros(3),
            np.eye(3),
            10.0,
            0.0,
            1,
            False,
        ),
    )

    def forbidden_socket(*_args, **_kwargs):
        raise AssertionError("UDP socket must not be opened for an unconverged solve")

    monkeypatch.setattr(ik_cli.socket, "socket", forbidden_socket)

    assert ik_cli.main() == 2


def test_camera_connection_applies_connect_and_send_timeouts(monkeypatch):
    calls = {}

    class FakeSocket:
        def settimeout(self, timeout):
            calls["send_timeout"] = timeout

    fake_socket = FakeSocket()

    def fake_create_connection(address, timeout):
        calls["address"] = address
        calls["connect_timeout"] = timeout
        return fake_socket

    monkeypatch.setattr(producer.socket, "create_connection", fake_create_connection)

    assert producer.connect_stream("192.0.2.10", 9999) is fake_socket
    assert calls == {
        "address": ("192.0.2.10", 9999),
        "connect_timeout": producer.CONNECT_TIMEOUT_SEC,
        "send_timeout": producer.SEND_TIMEOUT_SEC,
    }


def test_retired_shell_bridge_is_safe_and_disabled_by_default():
    source = (ROOT / "d1_sdk" / "src" / "bridge_simple.cpp").read_text(encoding="utf-8")
    cmake = (ROOT / "d1_sdk" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "system(" not in source
    assert "recvfrom(" not in source
    assert "D1_BUILD_LEGACY_BRIDGE" in cmake
    assert "D1_BUILD_LEGACY_BRIDGE\n    \"Build" in cmake
    assert "\n    OFF\n" in cmake


def test_active_bridge_accepts_configured_port_and_bounds_datagrams():
    source = (ROOT / "d1_sdk" / "src" / "multiple_joint_angle_control.cpp").read_text(encoding="utf-8")

    assert 'argument == "--port"' in source
    assert 'argument == "--bind"' in source
    assert "parsed < 1 || parsed > 65535" in source
    assert "MSG_TRUNC" in source
    assert "static_cast<std::size_t>(n) >= sizeof(buffer)" in source
    assert 'kDefaultBindAddress = "127.0.0.1"' in source
    assert "ValidateArmCommand(json_payload)" in source


def test_stream_endpoints_do_not_import_pickle():
    for relative_path in (
        "d1_hover_control_app.py",
        "test_tags3d_d435i_stream.py",
        "test_tags3d_live.py",
    ):
        source = (ROOT / relative_path).read_text(encoding="utf-8")
        assert "import pickle" not in source
        assert "pickle.loads" not in source
