# Legacy Docker setup (deprecated)

Drake's pre-built Docker images have been deprecated upstream. This repository previously used a `robotlocomotion/drake` base image and a VS Code Dev Container config.

The original Docker-based setup is preserved here for reference:

- `legacy/docker/Dockerfile`
- `legacy/docker/.devcontainer/devcontainer.json`

If you still choose to use Docker locally, note that this setup may stop working as the upstream images are removed or stop receiving updates. The recommended setup going forward is Drake via `pip` (see `../../README.md`).
