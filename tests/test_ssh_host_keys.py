from __future__ import annotations

import copy
from pathlib import Path

import paramiko
import pytest

import d1_hover_control_app as app


class FakeHostKeyClient:
    def __init__(self):
        self.keys = paramiko.HostKeys()

    def get_host_keys(self):
        return self.keys


def test_tofu_policy_persists_first_key_and_rejects_change(tmp_path: Path, capsys):
    known_hosts = tmp_path / "d1" / "known_hosts"
    first_key = paramiko.RSAKey.generate(1024)
    changed_key = paramiko.RSAKey.generate(1024)
    policy = app.PersistentTOFUHostKeyPolicy(paramiko, known_hosts)
    client = FakeHostKeyClient()

    policy.missing_host_key(client, "go2.example", first_key)

    assert known_hosts.is_file()
    first_connection_log = capsys.readouterr().out
    assert "Trusted first-seen key for go2.example" in first_connection_log
    assert "SHA256:" in first_connection_log
    assert str(known_hosts) in first_connection_log
    persisted = paramiko.HostKeys(str(known_hosts))
    assert persisted.lookup("go2.example")[first_key.get_name()] == first_key
    assert client.get_host_keys().lookup("go2.example")[first_key.get_name()] == first_key

    policy.missing_host_key(FakeHostKeyClient(), "go2.example", first_key)
    with pytest.raises(paramiko.BadHostKeyException):
        policy.missing_host_key(FakeHostKeyClient(), "go2.example", changed_key)


class FakeStream:
    def __init__(self, payload: bytes = b""):
        self.payload = payload
        self.channel = self

    def read(self):
        return self.payload

    def recv_exit_status(self):
        return 0


class FakeSSHClient:
    def __init__(self, *, connect_error=None):
        self.connect_error = connect_error
        self.system_loads = []
        self.user_loads = []
        self.policy = None
        self.connect_kwargs = None
        self.closed = False

    def load_system_host_keys(self, path=None):
        self.system_loads.append(path)

    def load_host_keys(self, path):
        self.user_loads.append(path)

    def set_missing_host_key_policy(self, policy):
        self.policy = policy

    def connect(self, **kwargs):
        self.connect_kwargs = kwargs
        if self.connect_error is not None:
            raise self.connect_error

    def exec_command(self, _command):
        return FakeStream(), FakeStream(b"ok\n"), FakeStream()

    def close(self):
        self.closed = True


def test_run_go2_command_loads_known_hosts_and_installs_persistent_policy(tmp_path: Path, monkeypatch):
    known_hosts = tmp_path / "known_hosts"
    keys = paramiko.HostKeys()
    keys.add("192.0.2.20", "ssh-rsa", paramiko.RSAKey.generate(1024))
    keys.save(str(known_hosts))
    client = FakeSSHClient()
    monkeypatch.setattr(paramiko, "SSHClient", lambda: client)
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    settings.update({"go2_host": "192.0.2.20", "go2_user": "robot", "go2_password": "secret"})

    result = app.run_go2_command(settings, "true", known_hosts)

    assert result == "ok"
    assert client.system_loads[0] is None
    assert client.user_loads == [str(known_hosts)]
    assert isinstance(client.policy, app.PersistentTOFUHostKeyPolicy)
    assert client.connect_kwargs["hostname"] == "192.0.2.20"
    assert client.closed is True


def test_run_go2_command_reports_changed_host_key_actionably(tmp_path: Path, monkeypatch):
    expected = paramiko.RSAKey.generate(1024)
    actual = paramiko.RSAKey.generate(1024)
    error = paramiko.BadHostKeyException("192.0.2.20", actual, expected)
    client = FakeSSHClient(connect_error=error)
    monkeypatch.setattr(paramiko, "SSHClient", lambda: client)
    settings = copy.deepcopy(app.DEFAULT_SETTINGS)
    settings["go2_host"] = "192.0.2.20"
    known_hosts = tmp_path / "known_hosts"

    with pytest.raises(RuntimeError, match="changed; refusing") as raised:
        app.run_go2_command(settings, "true", known_hosts)

    assert str(known_hosts) in str(raised.value)
    assert client.closed is True
