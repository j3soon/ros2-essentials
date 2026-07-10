#!/usr/bin/env python3
"""Generate a compact Markdown report from workspace smoke JSON manifests."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from datetime import datetime
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a Markdown proof report from smoke-test manifests."
    )
    parser.add_argument(
        "--summary",
        action="append",
        default=[],
        metavar="LABEL=PATH",
        help="Summary manifest to include. May be repeated.",
    )
    parser.add_argument(
        "--test-log",
        action="append",
        default=[],
        metavar="LABEL=PATH",
        help="Plain command log to include as passed evidence. May be repeated.",
    )
    parser.add_argument(
        "--note",
        action="append",
        default=[],
        help="Additional Markdown bullet under Notes. May be repeated.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Markdown report path to write.",
    )
    return parser.parse_args()


def split_label_path(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise ValueError(f"Expected LABEL=PATH, got: {value}")
    label, path = value.split("=", 1)
    if not label or not path:
        raise ValueError(f"Expected LABEL=PATH, got: {value}")
    return label, Path(path)


def load_summary(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def status_counts(results: list[dict]) -> dict[str, int]:
    return dict(Counter(result["status"] for result in results))


def workspace_count(results: list[dict]) -> int:
    return len({result["workspace"] for result in results})


def failed_results(results: list[dict]) -> list[dict]:
    return [result for result in results if result["status"] == "failed"]


def main() -> int:
    args = parse_args()
    summaries = [(label, path, load_summary(path)) for label, path in map(split_label_path, args.summary)]
    test_logs = list(map(split_label_path, args.test_log))

    lines = [
        "# Workspace Smoke Proof Report",
        "",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        "",
        "## Manifests",
    ]

    if not summaries and not test_logs:
        lines.append("- No manifests or logs were provided.")

    for label, path, data in summaries:
        results = data.get("results", [])
        lines.append(
            f"- `{label}`: `{path}`; results={len(results)}; "
            f"counts={status_counts(results)}; workspaces={workspace_count(results)}"
        )

    for label, path in test_logs:
        lines.append(f"- `{label}`: `{path}`")

    lines.extend(["", "## Failures"])
    failures = [(label, failed_results(data.get("results", []))) for label, _, data in summaries]
    failures = [(label, results) for label, results in failures if results]
    if not failures:
        lines.append("- No failed manifest results.")
    else:
        for label, results in failures:
            lines.append(f"- `{label}`: {len(results)} failed result(s)")
            for result in results[:10]:
                log_path = result.get("log_path") or "no log"
                check = result.get("check", result.get("kind", "unknown"))
                lines.append(
                    f"  - `{result['workspace']}` `{check}`: `{log_path}`"
                )
            if len(results) > 10:
                lines.append(f"  - ... {len(results) - 10} more")

    lines.extend(["", "## Notes"])
    if args.note:
        for note in args.note:
            lines.append(f"- {note}")
    else:
        lines.append("- No additional notes.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as error:
        print(f"error: {error}")
        raise SystemExit(2)
