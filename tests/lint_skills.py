"""Check that agent skill discovery links point to canonical skills."""

from pathlib import Path


repo = Path(__file__).resolve().parents[1]
canonical = repo / "skills"
shared_discovery = repo / ".agents" / "skills"
codex_discovery = repo / ".codex" / "skills"
claude_discovery = repo / ".claude" / "skills"

skill_names = {path.name for path in canonical.iterdir() if path.is_dir()}
assert skill_names, "No canonical skills found in skills/"

for name in skill_names:
    assert (canonical / name / "SKILL.md").is_file(), f"Missing SKILL.md: {name}"

for discovery in (shared_discovery, codex_discovery, claude_discovery):
    assert discovery.is_symlink(), f"Skill discovery directory is not a link: {discovery}"
    assert discovery.resolve(strict=True) == canonical.resolve(), f"Broken skill link: {discovery}"
