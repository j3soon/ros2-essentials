---
name: workspace-smoke-testing
description: Entry point for selecting and running reusable workspace-level Docker smoke tests and proof generation in this ROS2 robotics repository. Use when validating affected *_ws workspaces after Docker, script, package, simulator, docs, or skill changes; when a user asks whether to rebuild, run CLI checks, or capture GUI/Isaac/Gazebo proof; or when preparing a testing/proof PR.
---

# Workspace Smoke Testing

Use this as the router. For workspace-specific expectations, read the matching
`<workspace>-workspace-testing` skill first, then run the shared scripts under
`tests/workspace_smoke/`.

## Choose Scope

- Explicit workspace: pass `--workspace <name>_ws`.
- Changed files: pass `--changed-from <base> --changed-to HEAD`, or pipe paths
  with `--changed-files-from -`.
- Shared testing changes under `.agents/skills/`, `tests/workspace_smoke/`,
  `docker_modules/`, or setup scripts select all workspaces.

If the user has not chosen the validation depth and the run is expensive or
local GUI/GPU dependent, ask whether to rebuild images and whether GUI proof is
needed. If they only need a quick non-GUI check, prefer `image-cli`.

## Test Levels

- Build: `python3 tests/workspace_smoke/run.py --workspace <ws> --level build`
- CLI without Compose startup: `python3 tests/workspace_smoke/run.py --workspace <ws> --level image-cli`
- Runtime CLI through Compose: `python3 tests/workspace_smoke/run.py --workspace <ws> --level cli`
- Isaac headless renderer proof: `python3 tests/workspace_smoke/run.py --workspace <ws> --level isaac-visual`
- Documented GUI/Isaac proof: `python3 tests/workspace_smoke/doc_demo_smoke.py --workspace <ws> --summary-json tests/workspace_smoke/artifacts/<ws>-doc-demo.json`

Use `--no-gpu` only for config/build/image-cli checks. Use
`--disable-gpu-reservation` only as an explicit fallback when Compose runtime
checks are needed on a host with broken NVIDIA container runtime state.

## Proof Reports

Write JSON summaries with `--summary-json`. Convert saved manifests and
repository test logs into Markdown with:

```bash
python3 tests/workspace_smoke/report.py \
  --summary smoke=tests/workspace_smoke/artifacts/<summary>.json \
  --test-log test-all=tests/workspace_smoke/artifacts/<test-log>.log \
  --output tests/workspace_smoke/artifacts/proof-report.md
```

For GUI screenshots or recordings, use the shared `gui-proof-capture` skill and
`tests/workspace_smoke/proof_capture.py`. GUI proof captures the host X11
desktop/window after the container renders to the host display.

## Guardrails

- Keep heavy Docker/GPU checks out of `tests/test_all.sh`.
- Run `scripts/post_install.sh` when Docker module hard links may be stale.
- Use `--continue-on-failure` for broad proof collection.
- Always include log and artifact paths in the final report.
- Run `./tests/test_all.sh` after changing scripts, skills, workflows, or linted
  repository structure.
