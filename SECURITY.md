# Security Policy

## Supported versions

Security fixes are applied to the latest commit on `main`. No tagged releases are currently supported.

## Reporting a vulnerability

Use GitHub's private **Report a vulnerability** form in the repository Security tab. Do not disclose command-injection, network-protocol, credential, or robot-control issues in a public issue before a fix is available.

Include the affected commit, reproduction conditions, impact, and a minimal sanitized example. Remove credentials, private addresses, personal paths, and identifiable camera imagery.

## Deployment assumptions

This project controls physical hardware and is intended for an isolated, trusted robot network. The local web application is not an internet-facing service. Keep its HTTP and UDP listeners on loopback, restrict robot-side ports with host firewalls, and do not process frame data from unknown peers. The frame listener filters source addresses against `go2_host` by default; do not use `--stream-allow-any-peer` outside an isolated deployment. The web app's host, origin, JSON content-type, and per-process request-token checks mitigate browser request forgery; they do not provide user authentication or transport encryption.

Stored SSH credentials are local secrets. Prefer key-based authentication, protect local settings permissions, and never commit the generated settings file. The app logs and persists first-seen Go2 keys under `~/.config/d1-hover-control/known_hosts` and rejects changes. Trust on first use does not authenticate the initial connection, so compare the logged fingerprint independently when possible and verify the device before editing that file after a mismatch.
