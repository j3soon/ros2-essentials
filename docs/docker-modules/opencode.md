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

## Configuring Local Models

Follow the [OpenCode config locations](https://opencode.ai/docs/config/#locations) guide and edit `~/.config/opencode/opencode.json` directly.

The default `compose.yaml` mounts `${HOME}/docker/.config/opencode` to `/home/user/.config/opencode` and `${HOME}/docker/.local/share/opencode` to `/home/user/.local/share/opencode` so the installed binary and provider credentials can persist across containers.

For a local [llama.cpp](https://github.com/ggml-org/llama.cpp) endpoint, add a provider entry like this by downloading and running the setup helper script:

```sh
curl -fsSL https://raw.githubusercontent.com/j3soon/local-llm-notes/refs/heads/main/examples/basic-secure-api/scripts/setup_opencode.sh -o /tmp/setup_opencode.sh
chmod +x /tmp/setup_opencode.sh
/tmp/setup_opencode.sh
```

The script installs the necessary dependencies and sets the `OPENCODE_MODEL` environment variable to point to the local model. After running it, you can start OpenCode as usual and it will use the locally-installed model.

## Privacy

Note that using cloud-hosted models may allow your code to be used for training. See the OpenCode privacy policy for details: [privacy policy](https://opencode.ai/docs/zen/#privacy).

## References

- [OpenCode](https://opencode.ai/)
- [OpenCode install command](https://opencode.ai/)
- [OpenCode source code](https://github.com/anomalyco/opencode)
- [Dockerfile fragment reference](https://github.com/j3soon/dockerfile-fragments/tree/main/opencode)
