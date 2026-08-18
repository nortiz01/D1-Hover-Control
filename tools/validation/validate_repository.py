#!/usr/bin/env python3
"""Validate repository metadata and assets without robot hardware."""

from __future__ import annotations

import json
import re
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote

import yaml


ROOT = Path(__file__).resolve().parents[2]
MARKDOWN_LINK = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
SKIP_PARTS = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "build",
    "env",
    "install",
    "log",
    "logs",
    "venv",
}


def repository_files() -> tuple[Path, ...]:
    """Return tracked and non-ignored files, with an archive-safe fallback."""
    try:
        result = subprocess.run(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
            cwd=ROOT,
            check=True,
            capture_output=True,
        )
        return tuple(
            ROOT / entry.decode("utf-8")
            for entry in result.stdout.split(b"\0")
            if entry
        )
    except (FileNotFoundError, subprocess.CalledProcessError, UnicodeDecodeError):
        return tuple(
            path
            for path in ROOT.rglob("*")
            if path.is_file() and not SKIP_PARTS.intersection(path.relative_to(ROOT).parts)
        )


REPOSITORY_FILES = repository_files()


def validate_structured_files() -> int:
    checked = 0
    for path in REPOSITORY_FILES:
        if path.suffix == ".json":
            json.loads(path.read_text(encoding="utf-8"))
            checked += 1
        elif path.suffix in {".xml", ".urdf", ".launch"}:
            ET.parse(path)
            checked += 1
        elif path.suffix in {".yaml", ".yml"}:
            yaml.safe_load(path.read_text(encoding="utf-8"))
            checked += 1
    return checked


def validate_mesh_references() -> int:
    checked = 0
    for urdf_path in (path for path in REPOSITORY_FILES if path.suffix == ".urdf"):
        tree = ET.parse(urdf_path)
        for mesh in tree.findall(".//mesh"):
            reference = mesh.get("filename")
            if not reference:
                continue
            if reference.startswith("package://d1_550_description/"):
                relative = reference.removeprefix("package://d1_550_description/")
                target = ROOT / "d1_550_description" / relative
            else:
                target = urdf_path.parent / reference
            if not target.is_file():
                raise FileNotFoundError(f"Missing mesh referenced by {urdf_path}: {reference}")
            checked += 1
    return checked


def validate_markdown() -> int:
    checked = 0
    for path in (candidate for candidate in REPOSITORY_FILES if candidate.suffix == ".md"):
        text = path.read_text(encoding="utf-8")
        if text.count("```") % 2:
            raise ValueError(f"Unbalanced fenced code block in {path}")
        for raw_target in MARKDOWN_LINK.findall(text):
            target = raw_target.strip().split(maxsplit=1)[0].strip("<>")
            if not target or target.startswith(("#", "http://", "https://", "mailto:")):
                continue
            relative = unquote(target.split("#", 1)[0])
            if relative and not (path.parent / relative).resolve().exists():
                raise FileNotFoundError(f"Broken local link in {path}: {target}")
        checked += 1
    return checked


def validate_social_preview() -> None:
    path = ROOT / "docs" / "assets" / "social-preview.png"
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"Not a PNG file: {path}")
    width, height = struct.unpack(">II", data[16:24])
    if (width, height) != (1280, 640):
        raise ValueError(f"Social preview must be 1280x640, got {width}x{height}")


def main() -> int:
    structured = validate_structured_files()
    meshes = validate_mesh_references()
    markdown = validate_markdown()
    validate_social_preview()
    print(f"Validated {structured} structured files, {meshes} mesh references, and {markdown} Markdown files.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
