#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Create local environment folders/files used by tooling.
mkdir -p "${SCRIPT_DIR}/../.env"
mkdir -p "${SCRIPT_DIR}/../.env/.claude"
touch "${SCRIPT_DIR}/../.env/.claude.json"
mkdir -p "${SCRIPT_DIR}/../.env/.codex"

echo "Set up local environment files done."
