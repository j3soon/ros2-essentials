import os
import glob
import logging

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

logging.info("Checking `compose.yaml` files...")
for filename in glob.glob(f"{repo_dir}/**/compose*.yaml", recursive=True):
    logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
    with open(filename, "r") as f:
        content = f.read()
        if "version:" in content:
            # Ref: https://docs.docker.com/compose/compose-file/04-version-and-name/#version-top-level-element-optional
            raise ValueError(f"`version` should not exist since it's obsolete: '{filename}'")
