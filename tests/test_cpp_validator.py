from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_arm_command_validator_compiles_cleanly_and_rejects_unsafe_payloads(tmp_path: Path):
    compiler = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.skip("No C++ compiler is available in this environment")

    executable = tmp_path / ("arm_command_validator_test.exe" if Path(compiler).suffix.lower() == ".exe" else "arm_command_validator_test")
    source_dir = ROOT / "d1_sdk" / "src"
    compile_result = subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            f"-I{source_dir}",
            str(source_dir / "arm_command_validator.cpp"),
            str(ROOT / "d1_sdk" / "tests" / "arm_command_validator_test.cpp"),
            "-o",
            str(executable),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert compile_result.returncode == 0, compile_result.stdout + compile_result.stderr

    run_result = subprocess.run(
        [str(executable)],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert run_result.returncode == 0, run_result.stdout + run_result.stderr
