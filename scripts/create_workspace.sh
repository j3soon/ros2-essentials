#!/bin/bash -e

# Read the name of new workspace from the command line.
if [ $# == 1 ]; then
    NEW_WS_NAME_UNDERSCORE="$(echo $1 | sed "s/-/_/g")"
    NEW_WS_NAME_HYPHEN="$(echo $1 | sed "s/_/-/g")"
else
    echo "Error: Please specify the name of new workspace."
    echo "Usage: $0 <new_workspace_name>"
    exit 1
fi

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

# Get the directory of the template workspace.
TEMPLATE_WS_DIR="${SCRIPT_DIR}/../template_ws"

# Set the directory of the new workspace.
NEW_WS_DIR="${SCRIPT_DIR}/../${NEW_WS_NAME_UNDERSCORE}"

# Check whether the new workspace exists.
if [ -d "${NEW_WS_DIR}" ]; then
    echo "Error: Workspace ${NEW_WS_NAME_UNDERSCORE} already exists."
    exit 1
else
    echo "Creating new workspace in $(realpath ${NEW_WS_DIR}) ..."
fi

# Copy the template_ws to new workspace.
cp -r "${TEMPLATE_WS_DIR}" "${NEW_WS_DIR}"

# Remove the files that should not be copied.
rm -rf "${NEW_WS_DIR}/.vscode"
rm -rf "${NEW_WS_DIR}/build" "${NEW_WS_DIR}/install" "${NEW_WS_DIR}/log"
mkdir "${SCRIPT_DIR}/../docs/${NEW_WS_NAME_HYPHEN}"
echo "# ${NEW_WS_NAME_UNDERSCORE}" > "${SCRIPT_DIR}/../docs/${NEW_WS_NAME_HYPHEN}/README.md"

# Replace the "template_ws" with new workspace name in each file.
sed -i 's/"name": "Template",/"name": "TODO",/g'   "${NEW_WS_DIR}/.devcontainer/devcontainer.json"
sed -i "s/template_ws/${NEW_WS_NAME_UNDERSCORE}/g" "${NEW_WS_DIR}/.devcontainer/devcontainer.json"
sed -i "s/template-ws/${NEW_WS_NAME_HYPHEN}/g"     "${NEW_WS_DIR}/.devcontainer/devcontainer.json"
sed -i "s/template_ws/${NEW_WS_NAME_UNDERSCORE}/g" "${NEW_WS_DIR}/docker/compose.yaml"
sed -i "s/template-ws/${NEW_WS_NAME_HYPHEN}/g"     "${NEW_WS_DIR}/docker/compose.yaml"

echo "Done."
