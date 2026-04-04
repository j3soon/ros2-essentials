# OpenCode

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_opencode.sh)

OpenCode CLI for AI-assisted development directly inside workspace containers.

> See [Last tested](../last-tested.md) for the latest validation status.

To enable OpenCode CLI, set the `OPENCODE` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

> **Notice - Subscription required:** OpenCode is a third-party service. This module only installs the CLI. You still need to sign in with a supported provider such as GitHub Copilot, Anthropic, OpenAI, Google, or others supported by OpenCode.

## Installation

The module installs OpenCode CLI using the official installer from [https://opencode.ai/install](https://opencode.ai/install).

> The current installer adds the CLI binary under `~/.opencode/bin`, which is already available on the default container `PATH`.

## Usage

After building a workspace with OpenCode enabled, you can use the CLI inside the container:

```sh
# Start your workspace container
cd ~/ros2-essentials/template_ws/docker
docker compose up -d
docker exec -it ros2-template-ws bash

# Inside the container, verify OpenCode is installed
opencode --version

# Run OpenCode CLI
opencode
```

The first time you run OpenCode, it will prompt you to choose and authenticate a model provider.

## Authentication

The default `compose.yaml` mounts `${HOME}/docker/.opencode` to `/home/user/.opencode` and `${HOME}/docker/.local/share/opencode` to `/home/user/.local/share/opencode` so the installed binary and provider credentials can persist across containers.

## Privacy

Note that using cloud‑hosted models may allow your code to be used for training. See the OpenCode privacy policy for details: [privacy policy](https://opencode.ai/docs/zen/#privacy). Additionally, refer to `setup_opencode_llamacpp.sh` for local [llama.cpp](https://github.com/ggml-org/llama.cpp) configuration.

## References

- [OpenCode](https://opencode.ai/)
- [OpenCode install command](https://opencode.ai/)
- [OpenCode source code](https://github.com/anomalyco/opencode)
- [Dockerfile fragment reference](https://github.com/j3soon/dockerfile-fragments/tree/main/opencode)
