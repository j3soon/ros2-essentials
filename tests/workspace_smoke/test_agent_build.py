"""Opt-in Docker checks for conditional coding-agent release cache refresh."""

from __future__ import annotations

import os
import re
import subprocess
import threading
import unittest
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = REPO_ROOT / "tests/workspace_smoke/artifacts/review-122/agent-releases"
# The default context otherwise inherits an endpoint set through DOCKER_HOST.
LOCAL_DOCKER_ENV = dict(os.environ)
LOCAL_DOCKER_ENV.pop("DOCKER_HOST", None)
AGENTS = {
    "CLAUDE_CODE": ("claude-code", "claude_code", "claude-code-latest-version"),
    "CODEX": ("codex", "codex", "openai-codex-latest.json"),
    "OPENCODE": ("opencode", "opencode", "opencode-latest.json"),
    "PI": ("pi", "pi", "pi-coding-agent-latest.json"),
}


def agent_dockerfile() -> str:
    content = (REPO_ROOT / "template_ws/docker/Dockerfile").read_text()
    agent_layers = content.split("# Claude Code CLI configuration\n", 1)[1]
    agent_layers = agent_layers.split(
        "# TODO: Install more optional development tools", 1
    )[0]
    return "FROM ubuntu:22.04\nARG USERNAME=root\n" + agent_layers


def build(
    content: str,
    name: str,
    *,
    flags: dict[str, str] | None = None,
    modules: Path | None = None,
    image: str | None = None,
    no_cache: bool = False,
) -> str:
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    (ARTIFACTS / ".bashrc").write_text("# Disabled agent release cache marker.\n")
    dockerfile = ARTIFACTS / f"{name}.Dockerfile"
    dockerfile.write_text(content, encoding="utf-8")
    command = [
        "docker",
        "--context",
        "default",
        "buildx",
        "build",
        "--builder",
        "default",
        "--network=none",
        "--pull=false",
        "--progress=plain",
        "--build-context",
        f"docker_modules={modules or REPO_ROOT / 'docker_modules'}",
        "-f",
        str(dockerfile),
    ]
    if no_cache:
        command.append("--no-cache")
    if image:
        command.extend(["--load", "--tag", image])
    for key, value in (flags or {}).items():
        command.extend(["--build-arg", f"{key}={value}"])
    command.append(str(ARTIFACTS))
    log_path = ARTIFACTS / f"{name}.log"
    with log_path.open("w") as log:
        result = subprocess.run(
            command,
            cwd=REPO_ROOT,
            env=LOCAL_DOCKER_ENV,
            stdout=log,
            stderr=subprocess.STDOUT,
            timeout=120,
        )
    output = log_path.read_text()
    if result.returncode != 0:
        raise AssertionError(f"{name} failed. See {log_path}\n{output}")
    return output


@unittest.skipUnless(
    os.environ.get("WORKSPACE_SMOKE_DOCKER_TEST"), "opt-in Docker build"
)
class DisabledAgentBuildTests(unittest.TestCase):
    def test_disabled_agent_layers_build_without_network(self):
        # --network=none covers RUN, while BuildKit fetches remote ADD separately.
        # Make any release metadata ADD fail as it would during an endpoint outage.
        test_content = re.sub(
            r"https?://[^\s}]+",
            "http://127.0.0.1:1/unavailable-agent-release",
            agent_dockerfile(),
        )
        for index, value in enumerate([None, ""]):
            with self.subTest(value=value):
                flags = None if value is None else dict.fromkeys(AGENTS, value)
                output = build(
                    test_content, f"disabled-{index}", flags=flags, no_cache=True
                )
                self.assertEqual(output.count("Skipping "), 4)


@unittest.skipUnless(
    os.environ.get("WORKSPACE_SMOKE_DOCKER_TEST"), "opt-in Docker build"
)
class ReleaseCacheTests(unittest.TestCase):
    def test_only_enabled_endpoints_are_checked_and_changes_refresh_installation(self):
        versions = {name: "1.0.0" for name, _, _ in AGENTS.values()}
        requests: list[str] = []

        class MetadataHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                name = self.path.strip("/")
                requests.append(name)
                if name not in versions:
                    self.send_error(503, "disabled agent endpoint")
                    return
                body = (versions[name] + "\n").encode()
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("ETag", f'"{versions[name]}"')
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), MetadataHandler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        image = f"local/ros2-agent-release-smoke:{uuid.uuid4().hex}"
        modules = ARTIFACTS / "stub-modules"
        modules.mkdir(parents=True, exist_ok=True)
        for flag, (_, script, filename) in AGENTS.items():
            installer = modules / f"install_{script}.sh"
            installer.write_text(
                "#!/bin/sh\nset -e\n"
                f'case "${flag}" in YES|yes|Y|y)\n'
                f"  cp /tmp/{filename} /installed-{flag}\n"
                f"  date +%s%N > /install-token-{flag}\n"
                '  ;;\n*) echo "Skipping disabled agent" ;;\nesac\n',
                encoding="utf-8",
            )
            installer.chmod(0o755)
        port = server.server_address[1]
        content = agent_dockerfile()
        for flag, (name, _, _) in AGENTS.items():
            content = re.sub(
                rf"(?m)(ARG {flag}_RELEASE_SOURCE=\$\{{{flag}:\+)https?://[^}}]+",
                rf"\1http://127.0.0.1:{port}/{name}",
                content,
            )

        def installed(flag: str) -> list[str]:
            return subprocess.check_output(
                [
                    "docker",
                    "--context",
                    "default",
                    "run",
                    "--rm",
                    "--network=none",
                    "--entrypoint",
                    "sh",
                    image,
                    "-c",
                    f"cat /installed-{flag} /install-token-{flag}",
                ],
                env=LOCAL_DOCKER_ENV,
                text=True,
                timeout=30,
            ).splitlines()

        try:
            for flag, (name, _, _) in AGENTS.items():
                with self.subTest(agent=flag):
                    flags = {flag: "YES"}
                    requests.clear()
                    build(
                        content,
                        f"{name}-first",
                        flags=flags,
                        modules=modules,
                        image=image,
                    )
                    first = installed(flag)
                    self.assertEqual(first[0], "1.0.0")
                    self.assertTrue(requests)
                    self.assertEqual(set(requests), {name})
                    requests.clear()
                    build(
                        content,
                        f"{name}-unchanged",
                        flags=flags,
                        modules=modules,
                        image=image,
                    )
                    self.assertEqual(
                        installed(flag),
                        first,
                        "Unchanged metadata should reuse the install layer",
                    )
                    self.assertTrue(
                        requests,
                        "Enabled release metadata should be checked on cached builds",
                    )
                    self.assertEqual(set(requests), {name})
                    versions[name] = "2.0.0"
                    requests.clear()
                    build(
                        content,
                        f"{name}-updated",
                        flags=flags,
                        modules=modules,
                        image=image,
                    )
                    updated = installed(flag)
                    self.assertEqual(updated[0], "2.0.0")
                    self.assertNotEqual(
                        updated[1],
                        first[1],
                        "Changed metadata should rerun installation",
                    )
                    self.assertEqual(set(requests), {name})
            for value in ("yes", "Y", "y"):
                with self.subTest(enable_value=value):
                    requests.clear()
                    build(
                        content,
                        f"enabled-{value}",
                        flags=dict.fromkeys(AGENTS, value),
                        modules=modules,
                        image=image,
                    )
                    self.assertEqual(set(requests), set(versions))
                    for flag, (name, _, _) in AGENTS.items():
                        self.assertEqual(installed(flag)[0], versions[name])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
            subprocess.run(
                ["docker", "--context", "default", "image", "rm", image],
                env=LOCAL_DOCKER_ENV,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


if __name__ == "__main__":
    unittest.main()
