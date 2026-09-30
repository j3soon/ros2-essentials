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

# Install Node.js before the npm package. Keep this module usable when Codex is disabled.
sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg \
    && sudo rm -rf /var/lib/apt/lists/*
curl -fsSL https://deb.nodesource.com/setup_24.x | sudo -E bash -
sudo apt-get update && sudo apt-get install -y nodejs \
    && sudo rm -rf /var/lib/apt/lists/*
sudo npm install -g opencode-ai@latest

echo "OpenCode CLI installed successfully!"
echo "Version information:"
opencode --version

echo "OpenCode installation completed!"
