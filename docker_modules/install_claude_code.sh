#!/bin/bash
set -e

if [ "$CLAUDE_CODE" != "YES" ] && [ "$CLAUDE_CODE" != "yes" ] && [ "$CLAUDE_CODE" != "y" ] && [ "$CLAUDE_CODE" != "Y" ]; then
    echo "Skipping Claude Code installation (set CLAUDE_CODE to YES/yes/y/Y to enable)"
    exit 0
fi

# Ref: https://code.claude.com/docs/en/setup
# Install Claude Code CLI
# This script is intended to be run inside the Dockerfile during build.
echo "Installing Claude Code CLI"

# Install Claude Code CLI using official installer
curl -fsSL https://claude.ai/install.sh | bash

echo "Claude Code CLI installed successfully!"
echo "Version information:"
claude --version || true

echo "Claude Code installation completed!"
