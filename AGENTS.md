# Repository Guidelines

## Project Structure & Module Organization
- `*_ws/` are independent ROS2 workspaces. Each contains `docker/compose.yaml`, `docker/Dockerfile`, `.devcontainer/`, and `src/` (ROS2 packages built with colcon inside the container).
- `docker_modules/` holds shared install scripts exposed to workspace builds through Docker Compose `additional_contexts`.
- `docs/` is MkDocs content, including per-workspace docs under `docs/<workspace-name>/`.
- `scripts/` contains setup helpers (e.g., `post_install.sh`, `create_workspace.sh`).
- `tests/` contains lint-style checks for compose files, Dockerfiles, MkDocs, and workspace templates.

## Build, Test, and Development Commands
- `./scripts/post_install.sh`: refreshes local env files, shared cache volumes, and the optional Isaac Sim host link.
- `./scripts/post_install.sh --recreate-volumes --remove-containers`: recreates shared cache volumes when needed.
- `./scripts/enable_module.sh <MODULE>`: enable a module in the current workspace `docker/compose.yaml` (prompts for workspace/module selection if needed).
- `cd <workspace>/docker && docker compose build`: builds the workspace image.
- `cd <workspace>/docker && docker compose pull`: pulls a pre-built workspace image.
- `cd <workspace>/docker && docker compose up -d`: starts containers in the background.
- `cd <workspace>/docker && docker compose exec <service> bash`: opens a shell in the container.
- `./scripts/create_workspace.sh <new_workspace_name>`: scaffolds a new workspace from `template_ws`.
- `./tests/test_all.sh`: runs linting scripts for structure and config validation.

## Coding Style & Naming Conventions
- Workspaces must follow the `*_ws` naming pattern.
- Use `docker/compose.yaml` (not `compose.yml` or other variants).
- Keep required default files in each workspace: `.devcontainer/devcontainer.json`, `docker/Dockerfile`, `docker/compose.yaml`, `src/`, and `README.md`.
- Prefer USDA over USD for Omniverse/Isaac assets where possible.
- Keep comments and documentation concise and informative. Do not use em dashes or semicolons to join sentences.

## Shared Workflow
- Keep edits scoped to the request and preserve existing patterns.
- Run affected checks before finishing.
- Preserve each file's staged or unstaged state. Stage, unstage, or commit only when explicitly asked.
- Avoid trailing spaces and end files with a newline.
- Record durable, general guidance in the nearest relevant `AGENTS.md`.

## Testing Guidelines
- Primary checks are Python-based lint scripts executed via `./tests/test_all.sh`.
- GUI proof must identify the intended application window and reject captures when it is hidden or obscured.
- `lint_comp_template.py` treats `tests/diff_base/` as the canonical baseline; when `template_ws` intentionally changes, update `tests/diff_base/` and sync other workspaces as needed. Do not use `{PLACEHOLDER_MULTILINE}` in the baseline templates unless the user explicitly asks for it.
- You can skip workspaces by setting `IGNORED_WORKSPACES` (e.g., `export IGNORED_WORKSPACES="tmp_ws"`).

## Agent Skills
- Maintain skill content in `skills/`. The `.agents/skills`, `.codex/skills`, and `.claude/skills` directories are discovery links to it.
- Update a canonical skill and its references together. Keep discovery links valid.

## Commit & Pull Request Guidelines
- Branch naming: `feat/<name>` or `fix/<name>`.
- Keep commits focused so each change can be understood, validated, and reverted independently.
- Commit messages must follow Conventional Commits. Keep the body short, explain the rationale, and include sources when relevant.
- When `template_ws` changes require syncing other workspaces, make a separate minimal "unify" commit (preferred message: `feat: Unify workspaces style`).
- If code/content is copied, include source and commit permalink in the commit message.
- Check `git config user.name` and `git config user.email` before committing. Use the configured human identity as author and committer. If either is missing, ask the user. Never use a coding agent identity as author, committer, or co-author.
- For commits created by a coding agent, include a validation paragraph naming checks and results. End the body with a separate plain `by <Harness> (<Model>)` line using the actual harness and full canonical lowercase model slug, such as `by Codex (gpt-5.6-sol)`. Verify the active model before committing if its slug is unclear.
- The `by` line is the only agent attribution. Do not add co-author trailers or session links after it.
- Build multi-paragraph commit messages with separate `git commit -m` arguments. Do not embed escaped `\\n` sequences.
- After committing or rewriting a commit, inspect `git log -1 --format=fuller` to confirm paragraph breaks and the attribution as the final line.
- Avoid force-pushes once review starts.

## Configuration Tips
- Set `export USER_UID=$(id -u)` on the host to match container user permissions.
- Enable/disable modules via `build.args` in `docker/compose.yaml` (e.g., `CARTOGRAPHER: "YES"` or `CARTOGRAPHER: ""`). Rebuild affected images after changing a shared `docker_modules/` installer.
