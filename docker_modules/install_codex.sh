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

    # Install Node.js (LTS) via NodeSource to ensure modern JS features.
    # Ubuntu's default nodejs can be too old for the Codex CLI wrapper.
    sudo apt-get update && sudo apt-get install -y \
        ca-certificates \
        curl \
        gnupg \
        && sudo rm -rf /var/lib/apt/lists/*

    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt-get update && sudo apt-get install -y \
        nodejs \
        && sudo rm -rf /var/lib/apt/lists/*

    # Install Codex CLI globally
    sudo npm install -g @openai/codex

    echo "Codex CLI installed successfully!"
    echo "Version information:"
    node --version || true
    npm --version || true
    codex --version || true
fi

echo "Codex installation completed!"
