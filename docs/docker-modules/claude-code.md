# Claude Code

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_claude_code.sh)

Claude Code CLI for AI-assisted development directly inside workspace containers.

To enable Claude Code CLI, set the `CLAUDE_CODE` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

> **Notice - Subscription required:** Claude Code is a third-party service that requires an active Anthropic account or subscription to use. This module only installs the CLI; you need an active subscription to use it.

## Installation

The module automatically installs Claude Code CLI (latest version) using the official installer from [https://claude.ai/install.sh](https://claude.ai/install.sh).

> Alternatively, if you want to install the Claude Code extension for your favorite IDE (e.g., [Claude Code for VSCode](https://marketplace.visualstudio.com/items?itemName=anthropic.claude-code)), refer to the [Claude Code IDE extension](https://code.claude.com/docs/en/vs-code) for more details instead. This module don't need to be enabled if you just want to use the IDE extension.

## Usage

After building a workspace with Claude Code enabled, you can use the CLI inside the container:

```sh
# Start your workspace container
cd ~/ros2-essentials/template_ws/docker
docker compose up -d
docker exec -it ros2-template-ws bash

# Inside the container, verify Claude Code is installed
claude --version

# Run Claude Code CLI
claude
```

The first time you run Claude Code, you'll be prompted to authenticate with your Anthropic account.

If you want to run Claude Code CLI to run continuously without asking for permissions, make sure to comment out `privileged: true` in the `compose.yaml` file and restart the container. Then run:

```sh
claude --dangerously-skip-permissions
```

> This is not as secure as the [official container](https://code.claude.com/docs/en/devcontainer) due to the lack of firewall protection. But it should be fine for most cases as it's inside a container.

## Authentication

Claude Code caches login details locally at `~/.claude.json` and `~/.claude/`. The default `compose.yaml` mounts `<git_root_dir>/.env/.claude` to `/home/user/.claude` and `<git_root_dir>/.env/.claude.json` to `/home/user/.claude.json` so credentials can persist across containers.

## References

- [Set up Claude Code](https://code.claude.com/docs/en/setup)
- [Claude Code settings](https://code.claude.com/docs/en/settings)

For more discussions on AI coding assistants (such as Cursor etc.), please refer to [issue #102](https://github.com/j3soon/ros2-essentials/issues/102).
