#!/bin/bash
set -e

if [ -z "$CLAUDE_CODE" ]; then
    echo "Skipping Claude Code installation as CLAUDE_CODE is not set"
    exit 0
fi

# Ref: https://code.claude.com/docs/en/setup
# Install Claude Code CLI
# This script is intended to be run inside the Dockerfile during build.
if [ "$CLAUDE_CODE" = "YES" ]; then
    echo "Installing Claude Code CLI"

    # Install Claude Code CLI using official installer
    curl -fsSL https://claude.ai/install.sh | bash

    echo "Claude Code CLI installed successfully!"
    echo "Version information:"
    claude --version || true
fi

echo "Claude Code installation completed!"
