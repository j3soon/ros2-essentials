---
name: test-all-fixer
description: Run `/home/ros2-essentials/tests/test_all.sh` or `./tests/test_all.sh`, inspect the first failing lint script, apply the minimal repository fix, and repeat until the full suite passes. Use when asked to fix `tests/test_all.sh` or `tests/tests_all.sh` failures, clean up workspace-template drift, resolve repository structure/config lint errors, or verify the ROS2 workspaces repo after edits.
---

# Test All Fixer

Run the repository validation loop from the repo root. Fix the actual problem reported by the first failing check, then rerun the full suite until every test passes or an external blocker stops progress.

## Workflow

1. Locate the repository root by finding `tests/test_all.sh`.
2. Run `./tests/test_all.sh` from that root. If the user says `tests/tests_all.sh`, treat that as the real `tests/test_all.sh` path in this repo.
3. Read the first failing Python script in `tests/` and the file named in the traceback or error message.
4. Make the smallest change that satisfies the lint's intent. Do not weaken or disable tests unless the user explicitly asks for that.
5. Rerun `./tests/test_all.sh` after each fix. Keep iterating until it prints `All tests have passed.` or you hit a blocker that requires user input.
6. Report the failing check you fixed, the files changed, and whether the full suite is now clean.

## Guardrails

- Prefer full-suite reruns over single-test shortcuts. It is fine to run an individual failing script once while debugging, but always finish with `./tests/test_all.sh`.
- Preserve user changes. Avoid broad cleanup outside the failing check's scope.
- When the failure comes from `lint_comp_template.py`, inspect the matching file under `tests/diff_base/` before editing workspaces.
- Respect repo rules in `AGENTS.md`, especially `docker/compose.yaml` naming, required default files, and README/docs parity.
- Use `IGNORED_WORKSPACES` only when the user asks to exclude workspaces or when a temporary workspace would otherwise make the suite meaningless.
- If a fix to `template_ws` implies syncing many workspaces, keep the immediate test fix minimal unless the failure already proves broader drift that must be corrected.

## Failure Routing

Use [references/test-map.md](references/test-map.md) to map each failing test script to the files and invariants it checks.

## Output

State the exact command run, the failing script that was addressed, the files modified, and whether the final full-suite run passed.
