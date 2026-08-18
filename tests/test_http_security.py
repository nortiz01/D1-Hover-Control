from __future__ import annotations

import copy
import http.client
import json
import threading

import pytest

import d1_hover_control_app as app


TOKEN = "test-token-that-is-long-and-unpredictable"


def valid_post_headers(**overrides):
    values = {
        "host_header": "127.0.0.1:8080",
        "origin_header": "http://127.0.0.1:8080",
        "content_type": "application/json",
        "csrf_header": TOKEN,
        "csrf_token": TOKEN,
        "local_host": "127.0.0.1",
        "configured_host": "127.0.0.1",
        "expected_port": 8080,
    }
    values.update(overrides)
    return values


def test_host_validation_accepts_loopback_aliases_and_rejects_rebinding():
    assert app.validate_host_header(
        "localhost:8080",
        local_host="127.0.0.1",
        configured_host="127.0.0.1",
        expected_port=8080,
    ) == ("localhost", 8080)
    assert app.validate_host_header(
        "192.168.1.20:8080",
        local_host="192.168.1.20",
        configured_host="0.0.0.0",
        expected_port=8080,
    ) == ("192.168.1.20", 8080)

    for bad_host in (None, "attacker.example:8080", "127.0.0.1:8081", "user@127.0.0.1:8080"):
        with pytest.raises(app.RequestSecurityError):
            app.validate_host_header(
                bad_host,
                local_host="127.0.0.1",
                configured_host="127.0.0.1",
                expected_port=8080,
            )


def test_post_validation_accepts_only_json_same_origin_requests_with_token():
    app.validate_post_headers(**valid_post_headers())

    invalid_cases = [
        {"csrf_header": None},
        {"csrf_header": "wrong"},
        {"origin_header": None},
        {"origin_header": "null"},
        {"origin_header": "http://attacker.example:8080"},
        {"content_type": "application/x-www-form-urlencoded"},
        {"content_type": "application/json; charset=utf-8"},
        {"host_header": "attacker.example:8080"},
    ]
    for overrides in invalid_cases:
        with pytest.raises(app.RequestSecurityError):
            app.validate_post_headers(**valid_post_headers(**overrides))


def test_each_state_has_a_distinct_process_local_csrf_token():
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    app.normalize_settings(settings)
    first = app.SharedState(copy.deepcopy(settings))
    second = app.SharedState(copy.deepcopy(settings))

    assert len(first.csrf_token) >= 32
    assert first.csrf_token != second.csrf_token
    assert first.snapshot()["csrf_token"] == first.csrf_token


def test_http_handler_embeds_token_and_blocks_invalid_host_and_form_posts():
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    app.normalize_settings(settings)
    state = app.SharedState(settings)
    state.http_host = "127.0.0.1"

    server = app.ThreadingHTTPServer(("127.0.0.1", 0), app.RequestHandler)
    state.http_port = int(server.server_address[1])
    app.RequestHandler.state = state
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = http.client.HTTPConnection("127.0.0.1", state.http_port, timeout=2.0)
    try:
        connection.request("GET", "/")
        response = connection.getresponse()
        html = response.read().decode("utf-8")
        assert response.status == 200
        assert response.getheader("X-Frame-Options") == "DENY"
        assert response.getheader("Content-Security-Policy") == "frame-ancestors 'none'"
        assert f'content="{state.csrf_token}"' in html
        assert "__D1_CSRF_TOKEN__" not in html

        connection.request("GET", "/api/state")
        response = connection.getresponse()
        payload = json.loads(response.read())
        assert response.status == 200
        assert payload["csrf_token"] == state.csrf_token

        connection.request("GET", "/api/state", headers={"Host": f"attacker.example:{state.http_port}"})
        response = connection.getresponse()
        response.read()
        assert response.status == 403

        connection.request(
            "POST",
            "/api/zero_arm",
            body="dry_run=true",
            headers={
                "Host": f"127.0.0.1:{state.http_port}",
                "Origin": f"http://127.0.0.1:{state.http_port}",
                "Content-Type": "application/x-www-form-urlencoded",
            },
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 415

        connection.request(
            "POST",
            "/api/does-not-exist",
            body="{}",
            headers={
                "Host": f"127.0.0.1:{state.http_port}",
                "Origin": f"http://127.0.0.1:{state.http_port}",
                "Content-Type": "application/json",
                "X-CSRF-Token": state.csrf_token,
            },
        )
        response = connection.getresponse()
        response.read()
        assert response.status == 404
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
