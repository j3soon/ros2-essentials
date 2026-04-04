#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Create host-side environment folders/files used by tooling.
mkdir -p "${HOME}/docker/.claude"
touch "${HOME}/docker/.claude.json"
mkdir -p "${HOME}/docker/.codex"
mkdir -p "${HOME}/docker/.config/opencode"
mkdir -p "${HOME}/docker/.local/share/opencode"

echo "Set up host environment files under ${HOME}/docker done."
