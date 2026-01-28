# Claude Code

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_claude_code.sh)

Claude Code CLI for AI-assisted development directly inside workspace containers.

To enable Claude Code CLI, set the `CLAUDE_CODE` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

## Installation

The module automatically installs Claude Code CLI (latest version) using the official installer from [https://claude.ai/install.sh](https://claude.ai/install.sh).

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

If you want to run Claude Code CLI to run continuously without asking for permissions, make sure to comment out `privileged: true` in the `compose.yaml` file and restart the container. Then run:

```sh
claude --dangerously-skip-permissions
```

> This is not as secure as the [official container](https://code.claude.com/docs/en/devcontainer) due to the lack of firewall protection. But it should be fine for most cases as it's inside a container.

## Authentication

On first use, Claude Code will prompt you to authenticate with your Anthropic account. Follow the on-screen instructions to complete the authentication process.

## References

- [Set up Claude Code](https://code.claude.com/docs/en/setup)
- [Claude Code settings](https://code.claude.com/docs/en/settings)

For more discussions on AI coding assistants (such as Cursor etc.), please refer to [issue #102](https://github.com/j3soon/ros2-essentials/issues/102).
