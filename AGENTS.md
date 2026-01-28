# AGENTS.md

## Overview

ROS2 Humble multi-workspace repository for AMRs and robotic arm manipulators. Supports Isaac Sim/Lab/ROS for simulation-to-reality deployment. Targets amd64 (simulation+real) and arm64 (real only).

## Structure

- `*_ws/` - Independent ROS2 workspaces. Each contains:
  - `docker/compose.yaml` - Primary build/run config
  - `docker/Dockerfile` - Multi-arch image definition
  - `.devcontainer/` - VS Code Dev Container config
  - `src/` - ROS2 packages (built by colcon inside container)
- `docker_modules/` - Shared install scripts (hard-linked into workspaces via `post_install.sh`)
- `scripts/` - Setup and helper scripts
- `docs/` - MkDocs documentation
  - `*-ws/` - Documentation for each workspace
  - `docker-modules/` - Documentation for docker modules

## Build & Run

```sh
# After clone or pull, always run:
./scripts/post_install.sh

# Build and run a workspace:
cd <workspace>/docker
docker compose build
docker compose up -d
docker compose exec ros2 bash
```

## Key Conventions

- Run `./scripts/post_install.sh` after any repo change that affects docker_modules, such as switching git branches or adding new docker modules (creates hard links for docker_modules)
- `template_ws` is the base workspace; other workspaces share its Docker cache
- Modules are enabled/disabled via `build.args` in `compose.yaml` (e.g., `CARTOGRAPHER: "YES"`)
- Set `export USER_UID=$(id -u)` in host shell to match container user permissions

## Creating New Workspaces

Use `template_ws` as reference. Run `./scripts/create_workspace.sh <new_workspace_name>` and modify according to the pattern in existing workspaces.

## Linting

```sh
./tests/test_all.sh
```
