import os
import logging

git_committers_block = """
  - git-committers:
      repository: j3soon/ros2-essentials
      branch: main
"""

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking `mkdocs.yml` file...")
mkdocs_yml_path = f"{repo_dir}/docs/mkdocs.yml"
with open(mkdocs_yml_path, "r") as f:
    mkdocs_yml = f.read()

if git_committers_block not in mkdocs_yml:
    raise ValueError(f"`git-committers` block is missing in `{mkdocs_yml_path}`")
