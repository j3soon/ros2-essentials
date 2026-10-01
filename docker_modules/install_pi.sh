#!/bin/bash
set -e

if [ "$PI" != "YES" ] && [ "$PI" != "yes" ] && [ "$PI" != "y" ] && [ "$PI" != "Y" ]; then
    echo "Skipping Pi installation (set PI to YES/yes/y/Y to enable)"
    exit 0
fi

# Ref: https://pi.dev/docs/latest
# Ref: https://github.com/j3soon/dockerfile-fragments/tree/main/pi
echo "Installing Pi Coding Agent"

sudo apt-get update && sudo apt-get install -y ca-certificates curl gnupg \
    && sudo rm -rf /var/lib/apt/lists/*
curl -fsSL https://deb.nodesource.com/setup_24.x | sudo -E bash -
sudo apt-get update && sudo apt-get install -y nodejs \
    && sudo rm -rf /var/lib/apt/lists/*
sudo npm install -g --ignore-scripts @earendil-works/pi-coding-agent

echo "Pi Coding Agent installed successfully!"
pi --version
