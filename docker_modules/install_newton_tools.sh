#!/bin/bash
set -e

if [ "$NEWTON_TOOLS" != "YES" ] && [ "$NEWTON_TOOLS" != "yes" ] && [ "$NEWTON_TOOLS" != "y" ] && [ "$NEWTON_TOOLS" != "Y" ]; then
    echo "Skipping Newton Tools installation (set NEWTON_TOOLS to YES/yes/y to enable)"
    exit 0
fi

# Install Newton Tools converters
# This script is intended to be run inside the Dockerfile during build.
echo "Installing Newton Tools converters"
pip install urdf-usd-converter mujoco-usd-converter

echo "Newton Tools installation completed successfully!"
