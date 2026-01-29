#!/bin/bash
set -e

if [ -z "$CODEX" ]; then
    echo "Skipping Codex installation as CODEX is not set"
    exit 0
fi

# Ref: https://developers.openai.com/codex/cli/
# Install Codex CLI
# This script is intended to be run inside the Dockerfile during build.
if [ "$CODEX" = "YES" ]; then
    echo "Installing Codex CLI"

    # Install Node.js and npm via minimal apt install
    sudo apt-get update && sudo apt-get install -y \
        nodejs \
        npm \
        && sudo rm -rf /var/lib/apt/lists/*

    # Install Codex CLI globally
    sudo npm install -g @openai/codex

    echo "Codex CLI installed successfully!"
    echo "Version information:"
    codex --version || true
fi

echo "Codex installation completed!"
