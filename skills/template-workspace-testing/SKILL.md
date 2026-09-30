---
name: template-workspace-testing
description: Test and generate proof for template_ws. Use when validating workspace template build, Docker image CLI health, runtime checks, or documenting that no template-specific GUI demo exists.
---

# Template Workspace Testing

## Build

```bash
python3 tests/workspace_smoke/run.py --workspace template_ws --level build
```

## CLI

```bash
python3 tests/workspace_smoke/run.py --workspace template_ws --level image-cli
python3 tests/workspace_smoke/run.py --workspace template_ws --level cli
```

After a successful full build, add `--no-build` to image CLI or runtime CLI
checks to reuse the tested local image. This avoids restarting the large Isaac
installation build during a follow-up Compose check.

## GUI Proof

The template does not document a simulator-specific GUI demo, so the doc-demo
runner records this as skipped.

```bash
python3 tests/workspace_smoke/doc_demo_smoke.py --workspace template_ws \
  --summary-json tests/workspace_smoke/artifacts/template_ws-doc-demo.json
```
