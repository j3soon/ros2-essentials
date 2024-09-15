#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
cd $SCRIPT_DIR
# export IGNORED_WORKSPACES="temp_ws tmp_ws"
# Loop through all Python files in the current directory
for script in *.py; do
  echo "Running $script..."
  python3 "$script"
done
echo "All tests have passed."
