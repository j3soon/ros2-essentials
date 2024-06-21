#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"

RULE_FILENAME="60-kobuki.rules"

# Check the udev rule file exists.
if [ ! -f "${SCRIPT_DIR}/${RULE_FILENAME}" ]; then
    echo "${SCRIPT_DIR}/${RULE_FILENAME} not found."
    exit 1
fi

# Check whether the udevadm command exists.
# Reference: https://stackoverflow.com/q/592620
if ! command -v udevadm &> /dev/null ; then
    echo "udevadm command not found."
    echo "Installing udevadm ..."
    sudo apt-get update && sudo apt-get install -y udev
fi

# Copy the udev rule file to the /etc/udev/rules.d directory.
if [ -f "/etc/udev/rules.d/${RULE_FILENAME}" ]; then
    echo "/etc/udev/rules.d/${RULE_FILENAME} already exists."
    read -s -n 1 -p "Overwriting ? (Y/n) " answer
    echo
    if [ "${answer}" != "${answer#[Nn]}" ]; then
        echo "Keeping the existing file."
    else
        sudo cp -f "${SCRIPT_DIR}/${RULE_FILENAME}" "/etc/udev/rules.d/${RULE_FILENAME}"
        echo "Overwritten the existing file."
    fi
else
    sudo cp -f "${SCRIPT_DIR}/${RULE_FILENAME}" "/etc/udev/rules.d/${RULE_FILENAME}"
    echo "Copied the udev rule file."
fi

echo "Done."