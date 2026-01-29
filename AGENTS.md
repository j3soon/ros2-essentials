# Repository Guidelines

## Project Structure & Module Organization
- `*_ws/` are independent ROS2 workspaces. Each contains `docker/compose.yaml`, `docker/Dockerfile`, `.devcontainer/`, and `src/` (ROS2 packages built with colcon inside the container).
- `docker_modules/` holds shared install scripts that are hard-linked into each workspace via `./scripts/post_install.sh`.
- `docs/` is MkDocs content, including per-workspace docs under `docs/<workspace-name>/`.
- `scripts/` contains setup helpers (e.g., `post_install.sh`, `create_workspace.sh`).
- `tests/` contains lint-style checks for compose files, Dockerfiles, MkDocs, and workspace templates.

## Build, Test, and Development Commands
- `./scripts/post_install.sh` (or `-f` to force): refreshes docker module hard links; run after any repo change or branch switch.
- `cd <workspace>/docker && docker compose build`: builds the workspace image.
- `cd <workspace>/docker && docker compose up -d`: starts containers in the background.
- `cd <workspace>/docker && docker compose exec <service> bash`: opens a shell in the container.
- `./scripts/create_workspace.sh <new_workspace_name>`: scaffolds a new workspace from `template_ws`.
- `./tests/test_all.sh`: runs linting scripts for structure and config validation.

## Coding Style & Naming Conventions
- Workspaces must follow the `*_ws` naming pattern.
- Use `docker/compose.yaml` (not `compose.yml` or other variants).
- Keep required default files in each workspace: `.devcontainer/devcontainer.json`, `docker/Dockerfile`, `docker/compose.yaml`, `src/`, and `README.md`.
- Prefer USDA over USD for Omniverse/Isaac assets where possible.

## Testing Guidelines
- Primary checks are Python-based lint scripts executed via `./tests/test_all.sh`.
- You can skip workspaces by setting `IGNORED_WORKSPACES` (e.g., `export IGNORED_WORKSPACES="tmp_ws"`).

## Commit & Pull Request Guidelines
- Open and self-assign a GitHub issue before starting.
- Branch naming: `feat/<name>` or `fix/<name>`.
- Commit messages must follow Conventional Commits and include rationale and sources when relevant.
- If code/content is copied, include source and commit permalink in the commit message.
- Add `Co-authored-by` lines for contributors who helped; avoid force-pushes once review starts.

## Configuration Tips
- Set `export USER_UID=$(id -u)` on the host to match container user permissions.
- Enable/disable modules via `build.args` in `docker/compose.yaml` (e.g., `CARTOGRAPHER=ON`).
