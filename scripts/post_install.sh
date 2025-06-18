#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR"

./setup_docker_modules_link.sh
./setup_docs_link.sh
./setup_isaac_link.sh

echo "Post Install Done."
