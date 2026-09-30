# Test Map

- `lint_comp_template.py`: Enforce that workspace defaults match `tests/diff_base/` templates, with placeholder support. Check the matching `tests/diff_base/` file first, confirm whether `template_ws` is the intended new baseline, and do not use `{PLACEHOLDER_MULTILINE}` unless the user explicitly asks for it.
- `lint_compose.py`: Validate `*_ws/docker/compose.yaml` files. Remove obsolete `version:` keys and unresolved `PLACEHOLDER` text.
- `lint_dockerfile.py`: Validate `*_ws/docker/Dockerfile*`. Reject hard-coded `ros-humble-`, `$USER_GID`, standalone `apt-get update/install`, `$HOME`, and `PLACEHOLDER`.
- `lint_filenames.py`: Ensure every `*_ws` has the required default files and none of the obsolete paths like `docker/compose.yml`.
- `lint_gitignore.py`: Validate workspace `.gitignore` files. Reject obsolete `gazebo`, `docker/cache`, and `PLACEHOLDER` entries.
- `lint_mkdocs.py`: Ensure `docs/mkdocs.yml` contains the required `git-committers` block.
- `lint_readme.py`: Validate workspace `README.md` files, reject `ros2-agv-essentials` and `PLACEHOLDER`, and enforce root `README.md == docs/index.md`.
- `lint_workflows.py`: Reject `.github/workflows/*.yml` files and most `master` branch references in workflow YAML.
