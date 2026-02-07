#!/bin/bash
set -e

if [ -z "$NEWTON_TOOLS" ]; then
    echo "Skipping Newton Tools installation as NEWTON_TOOLS is not set"
    exit 0
fi

# Install Newton Tools converters
# This script is intended to be run inside the Dockerfile during build.
if [ "$NEWTON_TOOLS" = "YES" ]; then
    echo "Installing Newton Tools converters"
    pip install urdf-usd-converter mujoco-usd-converter
fi

echo "Newton Tools installation completed successfully!"
