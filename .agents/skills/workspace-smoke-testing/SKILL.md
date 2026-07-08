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
- Isaac Lab deformable smoke: `python3 tests/workspace_smoke/run.py --workspace <ws> --level isaac-lab-deformable`
- Documented GUI/Isaac proof: `python3 tests/workspace_smoke/doc_demo_smoke.py --workspace <ws> --summary-json tests/workspace_smoke/artifacts/<ws>-doc-demo.json`

For Compose-backed smoke paths, always build locally and refuse registry pulls:
`docker compose up -d --build --pull never`, then
`docker compose exec <service> bash -lc ...`. Keep direct `docker run` only for
cheap image CLI and Docker GPU preflight checks, and require those images to
already exist locally.

For documented Isaac joint-command motion proofs, capture the whole sequence:
open the stage with the timeline paused, validate the expected robot prim,
start host X11 recording, trigger Isaac timeline Play, wait for ROS topics and
scene stabilization, then publish the documented `/joint_command`. The
recording must include the start of simulation and pre-command frames, not only
the post-command motion.

For Isaac Lab deformable-object Kit GUI proof, run
`tests/workspace_smoke/isaac_lab_deformable_gui.sh --detach`, follow the printed
log path, and wait for `Registered backend 'kit'`, `[INFO]: Setup complete...`,
and the first `Root position (in world)` line. `Setup complete` can still
precede the first rendered viewport frame. Always take and inspect a check
screenshot before recording; if it shows a black viewport, wait until the
orange deformable objects render, then record. Confirm MP4 proof by extracting a
frame from the recording and verifying it shows the deformable scene. The Kit
GUI tutorial must run with a TTY; use `docker compose exec` from an interactive
terminal, or `script -qefc ... /dev/null` from non-interactive smoke runners.
For automated checks, poll the log inside the Compose service rather than the
outer PTY transcript; the host transcript can lag or remain attached after the
container has already emitted root-position lines.
Expect first Compose-based Kit launches to take minutes while Isaac/RTX/cache
startup work completes; compare `Simulation App Starting` to
`Simulation App Startup Complete` before calling it a deformable simulation hang.

For screenshot/recording proof requests, prefer the direct GUI capture path:
launch the workspace GUI, wait for a visible readiness marker/window, then use
`gui-proof-capture` / `proof_capture.py` to capture the host X11 display. Treat
`isaac-visual` as a deeper renderer smoke test, not the default proof path; use
it only when the user specifically asks for a headless render, deterministic
rendered scene, or simulator image output.

For Isaac GUI proof scenes, run
`tests/workspace_smoke/isaac_gui_proof_scene.py` inside the workspace container
and wait for `ISAAC_GUI_SCENE_READY` before calling `proof_capture.py`. The
helper authors a cube, ground, lights, and camera view, then keeps Isaac open
for host-side screenshot or recording capture.

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

## Reusable Test Logic

Treat `tests/workspace_smoke/artifacts/` as the normal output location for logs,
screenshots, recordings, and temporary proof data. Avoid `/tmp` for validation
work because a crash or reboot can erase it; use `/tmp` only when a tool
requires it, and immediately copy any useful result back into the repository's
artifact tree. Do not leave reusable validation scripts in output-only or
ephemeral locations.

When a docker-module or workspace validation needs a repeatable helper, promote
it to a git-tracked path before finishing:

- General repo helpers: `tests/workspace_smoke/`
- Shared Python utility code: `tests/workspace_smoke/lib/`
- Docker-module-specific helpers: `tests/workspace_smoke/modules/<module>/`
- Workspace-specific helpers: `tests/workspace_smoke/workspaces/<workspace>/`

If a prompt teaches a reusable workflow rule, update the relevant skill in
`.agents/skills/`. Skills should stay concise: point to the tracked script,
state readiness markers, capture/report expectations, and avoid embedding large
script bodies.

## Guardrails

- Keep heavy Docker/GPU checks out of `tests/test_all.sh`.
- Run `scripts/post_install.sh` when Docker module hard links may be stale.
- Use `--continue-on-failure` for broad proof collection.
- Always include log and artifact paths in the final report.
- Before finishing, check whether any ad hoc command or script should become a
  tracked helper or skill instruction for future reuse.
- Run `./tests/test_all.sh` after changing scripts, skills, workflows, or linted
  repository structure.
