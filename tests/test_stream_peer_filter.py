from __future__ import annotations

import copy
import socket

import pytest

import d1_hover_control_app as app


def new_state() -> app.SharedState:
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    app.normalize_settings(settings)
    settings["go2_host"] = "go2.example"
    return app.SharedState(settings)


def test_resolve_stream_peer_addresses_normalizes_and_deduplicates(monkeypatch):
    answers = [
        (socket.AF_INET, socket.SOCK_STREAM, 6, "", ("192.0.2.20", 0)),
        (socket.AF_INET6, socket.SOCK_STREAM, 6, "", ("::ffff:192.0.2.20", 0, 0, 0)),
        (socket.AF_INET6, socket.SOCK_STREAM, 6, "", ("2001:db8::20", 0, 0, 0)),
    ]
    monkeypatch.setattr(app.socket, "getaddrinfo", lambda *_args, **_kwargs: answers)

    assert app.resolve_stream_peer_addresses("go2.example") == {
        "192.0.2.20",
        "2001:db8::20",
    }


def test_resolve_stream_peer_addresses_fails_without_usable_answer(monkeypatch):
    answers = [(socket.AF_UNSPEC, socket.SOCK_STREAM, 6, "", ("not-an-address", 0))]
    monkeypatch.setattr(app.socket, "getaddrinfo", lambda *_args, **_kwargs: answers)

    with pytest.raises(OSError, match="No IP addresses resolved"):
        app.resolve_stream_peer_addresses("go2.example")


def test_frame_receiver_accepts_only_resolved_configured_host(monkeypatch):
    state = new_state()
    receiver = app.FrameReceiver(state, "127.0.0.1", 9999)
    monkeypatch.setattr(
        app,
        "resolve_stream_peer_addresses",
        lambda hostname: {"192.0.2.20", "2001:db8::20"} if hostname == "go2.example" else set(),
    )

    allowed, allowed_reason = receiver.peer_is_allowed("::ffff:192.0.2.20")
    rejected, rejected_reason = receiver.peer_is_allowed("192.0.2.99")

    assert allowed is True
    assert "configured Go2 host" in allowed_reason
    assert rejected is False
    assert "192.0.2.99" in rejected_reason
    assert "192.0.2.20" in rejected_reason


def test_frame_receiver_fails_closed_when_configured_host_cannot_resolve(monkeypatch):
    state = new_state()
    receiver = app.FrameReceiver(state, "127.0.0.1", 9999)

    def fail_resolution(_hostname):
        raise socket.gaierror("name lookup failed")

    monkeypatch.setattr(app, "resolve_stream_peer_addresses", fail_resolution)

    allowed, reason = receiver.peer_is_allowed("192.0.2.20")

    assert allowed is False
    assert "cannot verify configured Go2 host" in reason
    assert "name lookup failed" in reason


def test_allow_any_peer_opt_out_skips_resolution(monkeypatch):
    state = new_state()
    receiver = app.FrameReceiver(state, "127.0.0.1", 9999, allow_any_peer=True)
    monkeypatch.setattr(
        app,
        "resolve_stream_peer_addresses",
        lambda _hostname: pytest.fail("opt-out should not perform DNS resolution"),
    )

    allowed, reason = receiver.peer_is_allowed("198.51.100.40")

    assert allowed is True
    assert "filtering disabled" in reason
    with state.lock:
        assert state.stream_allow_any_peer is True


def test_stream_peer_cli_opt_out_is_explicit_and_documented():
    parser = app.build_argument_parser()

    assert parser.parse_args([]).stream_allow_any_peer is False
    assert parser.parse_args(["--stream-allow-any-peer"]).stream_allow_any_peer is True
    help_text = parser.format_help()
    assert "--stream-allow-any-peer" in help_text
    assert "configured Go2 host" in help_text
