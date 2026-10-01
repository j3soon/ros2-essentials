# Pi Coding Agent

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_pi.sh)

Pi is a terminal coding agent available inside workspace containers.

> See [Last tested](../last-tested.md) for the latest validation status.

To enable Pi, set the `PI` build argument to `YES` in a workspace's `docker/compose.yaml`, then rebuild its image. `template_ws` enables it by default.

The module installs Node.js 24 and the latest `@earendil-works/pi-coding-agent` npm package. Use `YES` to enable this agent and `""` to disable it. Enabled agents check release metadata during builds so a new release refreshes the install layer. Disabled agents use a local cache marker and skip the release download.

## Usage

```sh
cd ~/ros2-essentials/template_ws/docker
docker compose up -d
docker exec -it ros2-template-ws bash
pi --version
pi
```

Use `/login` in Pi to authenticate with a supported provider. For a one-shot request, run `pi -p "Summarize this repository"`.

The default Compose configuration mounts `${HOME}/docker/.pi/agent` at `/home/user/.pi/agent` to persist authentication, settings, sessions, and installed Pi packages. `./scripts/post_install.sh` creates the host directory.

## References

- [Pi documentation](https://pi.dev/docs/latest)
- [Pi Dockerfile fragment](https://github.com/j3soon/dockerfile-fragments/tree/main/pi)
