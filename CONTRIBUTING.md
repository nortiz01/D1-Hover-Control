# Contributing

Contributions are welcome when they preserve the project's hardware-safety boundaries and third-party notices.

## Development workflow

1. Create a focused branch from `main`.
2. Install the laptop dependencies in a virtual environment.
3. Make the smallest change that solves the problem.
4. Run the hardware-independent checks below.
5. Open a pull request describing validation, hardware impact, and any remaining limitations.

```bash
python -m pip install -r requirements-laptop.txt PyYAML pytest
python -m compileall -q \
  d1_hover_control_app.py d1_kinematics.py d1_stream_protocol.py \
  send_d1_550_ik_to_arm.py test_d1_550_ik.py \
  test_id0_apriltag_to_d1_hover.py test_tags3d_d435i_stream.py \
  test_tags3d_live.py d1_sdk/src/ik_solver.py tests tools
python tools/validation/validate_repository.py
python -m pytest -q tests
```

The Unitree SDK2 bridge and physical robot cannot be exercised in hosted CI. Clearly distinguish simulation, dry-run, and supervised hardware validation in pull requests.

## Robot safety

- Begin with dry-run behavior and keep the arm workspace clear.
- Do not weaken target-age, range, convergence, or command-serialization guards.
- Treat camera calibration, joint limits, and safe motion thresholds as installation-specific.
- Stop the system before changing network, calibration, or bridge settings.

## Security and privacy

- Never commit settings containing passwords or site-specific calibration data.
- Sanitize IP addresses, usernames, logs, screenshots, and camera frames.
- Do not expose the control API or frame receiver to an untrusted network.
- Report vulnerabilities using the process in [SECURITY.md](SECURITY.md).

## Licensing

The repository owner has not yet selected a license for project-owned code. Discuss substantial contributions in an issue before investing work, because accepting a patch does not by itself grant downstream reuse rights. Do not add or relicense third-party SDK material without confirmed redistribution terms, and preserve the provenance documented in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
