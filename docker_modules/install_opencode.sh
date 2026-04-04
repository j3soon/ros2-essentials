#!/bin/bash
set -e

if [ "$OPENCODE" != "YES" ] && [ "$OPENCODE" != "yes" ] && [ "$OPENCODE" != "y" ] && [ "$OPENCODE" != "Y" ]; then
    echo "Skipping OpenCode installation (set OPENCODE to YES/yes/y/Y to enable)"
    exit 0
fi

# Ref: https://opencode.ai/
# Ref: https://github.com/anomalyco/opencode
# Install OpenCode CLI
# This script is intended to be run inside the Dockerfile during build.
echo "Installing OpenCode CLI"

# The official installer places the binary in ~/.opencode/bin by default.
curl -fsSL https://opencode.ai/install | bash

echo "OpenCode CLI installed successfully!"
echo "Version information:"
opencode --version || true

echo "OpenCode installation completed!"
