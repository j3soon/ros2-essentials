# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 Humble multi-workspace repository for Autonomous Mobile Robots (AMRs) and robotic arm manipulators. Supports Isaac Sim/Lab/ROS for simulation-to-reality deployment. Targets amd64 (simulation+real) and arm64 (real only).

## Build & Run Commands

```sh
# After clone, or when local env files/volumes need refresh:
./scripts/post_install.sh

# Recreate shared cache volumes if needed:
./scripts/post_install.sh --recreate-volumes --remove-containers

# Build and run a workspace:
cd <workspace_name>/docker
docker compose build
docker compose up -d
docker compose exec <service_name> bash

# Pull pre-built images (faster than building):
cd <workspace_name>/docker
docker compose pull

# Create a new workspace:
./scripts/create_workspace.sh <new_workspace_name>

# Run linting tests:
./tests/test_all.sh
```

## Architecture

### Workspace Structure (`*_ws/`)
Each workspace is independent and follows this pattern:
- `docker/compose.yaml` - Primary build/run configuration with module args
- `docker/Dockerfile` - Multi-arch image definition (copies scripts from the `docker_modules` build context)
- `.devcontainer/` - VS Code Dev Container configuration
- `src/` - ROS2 packages (colcon builds these inside container)

### Docker Modules (`docker_modules/`)
Shared install scripts (e.g., `install_ros.sh`, `install_isaac_sim.sh`) exposed to workspace builds through the `docker_modules` Docker Compose `additional_contexts` entry.

Modules are enabled/disabled via `build.args` in `compose.yaml`:
```yaml
args:
  CARTOGRAPHER: "YES"  # or "" to disable
  RTABMAP: "YES"
  ISAAC_SIM_VERSION: "5.1.0"  # or "" to skip
  ISAAC_LAB_VERSION: "2.3.2"
```

### Key Files
- `template_ws/` - Base workspace used as reference for creating new workspaces
- `docs/` - MkDocs documentation source (each workspace has `docs/<workspace-name>/`)
- `scripts/setup_docker_volume.sh` - Sets up shared Docker volumes

## Key Conventions

- Docker module changes are picked up through the named `docker_modules` build context; rebuild affected images after module changes.
- Set `export USER_UID=$(id -u)` in host shell for container user permissions
- Workspaces share Docker volumes: `ros2-gazebo-cache`, `ros2-isaac-sim-cache`, `ros2-isaac-ros-assets`
- `template_ws` Docker cache is shared by other workspaces to save build time

## Linting

The `tests/` directory contains Python linting scripts:
- `lint_compose.py` - Validates compose.yaml files
- `lint_dockerfile.py` - Validates Dockerfiles
- `lint_comp_template.py` - Checks workspace compliance with template
- `lint_filenames.py` - Validates file naming conventions
- `lint_gitignore.py` - Checks gitignore consistency
- `lint_workflows.py` - Validates GitHub Actions workflows
- `lint_mkdocs.py` - Validates documentation structure
- `lint_readme.py` - Checks README files

Set `IGNORED_WORKSPACES` env var to skip specific workspaces during linting.
