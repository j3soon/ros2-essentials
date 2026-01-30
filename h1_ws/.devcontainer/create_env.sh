#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
DOCKER_DIR="${SCRIPT_DIR}/../docker"

# Normally, USER_UID is set during repo setup and works fine with the Docker CLI. 
# We need to manually write to .env because some devcontainer versions do not source ~/.bashrc,
# leaving the environment variables undeclared.
touch "${DOCKER_DIR}/.env"
awk '!/^USER_UID=/' "${DOCKER_DIR}/.env" > "${DOCKER_DIR}/.env.tmp"
echo "USER_UID=$(id -u)" >> "${DOCKER_DIR}/.env.tmp"
mv "${DOCKER_DIR}/.env.tmp" "${DOCKER_DIR}/.env"
