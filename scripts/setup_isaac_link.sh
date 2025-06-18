#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR/.."

rm _isaac_sim 2>/dev/null || true
ln -s ~/isaacsim _isaac_sim

echo "Set up Isaac link done."
