#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

cd "$SCRIPT_DIR/.."

workspaces=(
    "template_ws"
    "orbslam3_ws"
    "rtabmap_ws"
    "ros1_bridge_ws"
    "cartographer_ws"
    "husky_ws"
    "kobuki_ws"
    "vlp_ws"
    "gazebo_world_ws"
    "aloha_ws"
)

# Link docs index
rm docs/index.md || true
ln "${PWD}/README.md" docs/index.md

# Loop through each workspace and link its docs
for ws in "${workspaces[@]}"
do
    ws_hyphen="$(echo $ws | sed "s/_/-/g")"
    rm "${ws}/docs" || true
    ln -s "${PWD}/docs/${ws_hyphen}" "${ws}/docs"
done

echo "Done."
