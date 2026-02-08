# Codex

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_codex.sh)

Codex CLI for AI-assisted development directly inside workspace containers.

To enable Codex CLI, set the `CODEX` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

> **Notice - Subscription required:** Codex is a third-party service that requires an active OpenAI account or subscription to use. This module only installs the CLI; you need an active subscription to use it.

## Installation

The module installs the Codex CLI (latest version) via npm during the Docker image build.

> Alternatively, if you want to install the Codex extension for your favorite IDE (e.g., [Codex for VSCode](https://marketplace.visualstudio.com/items?itemName=openai.chatgpt)), refer to the [Codex IDE extension](https://developers.openai.com/codex/ide/) for more details instead. This module doesn't need to be enabled if you just want to use the IDE extension.

## Usage

After building a workspace with Codex enabled, you can use the CLI inside the container:

```sh
# Start your workspace container
cd ~/ros2-essentials/template_ws/docker
docker compose up -d
docker exec -it ros2-template-ws bash

# Inside the container, verify Codex is installed
codex --version

# Login with device code authentication
codex login --device-auth

# Run Codex CLI
codex
```

The first time you run Codex, you'll be prompted to sign in with your ChatGPT account or an API key.

If you want to run Codex CLI to run continuously without asking for permissions, make sure to comment out `privileged: true` in the `compose.yaml` file and restart the container. Then run:

```sh
codex --yolo
```

> This is not the most secure way to run Codex CLI, but it's often good enough as it's inside a container.

## Authentication

Codex caches login details locally at `~/.codex/auth.json`. The default `compose.yaml` mounts `<git_root_dir>/.env/.codex` to `/home/user/.codex` so credentials can persist across containers.

## References

- [Codex CLI](https://developers.openai.com/codex/cli)
- [Authentication](https://developers.openai.com/codex/auth)
- [Codex overview](https://openai.com/codex)
- [Config basics](https://developers.openai.com/codex/config-basic/)
- [Security](https://developers.openai.com/codex/security/)
- [Codex Source Code on GitHub](https://github.com/openai/codex)
