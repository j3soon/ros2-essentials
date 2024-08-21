import difflib
import glob
import logging
import os
import re
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

def compare_file_with_template(filepath, ignored_workspaces=[]):
    logging.info(f"Checking if '{filepath}' matches the template...")
    template_path = f"{repo_dir}/tests/diff_base/{filepath}"
    template = Path(template_path).read_text().splitlines(keepends=True)  # keepends to preserve trailing newlines
    for filename in glob.glob(f"{repo_dir}/*_ws/{filepath}"):
        # Skip certain cases intentionally
        if any(ws in filename for ws in ignored_workspaces):
            continue
        logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
        content = Path(filename).read_text().splitlines(keepends=True)
        diff = list(filter(lambda x: x.startswith('- ') or x.startswith('+ '), difflib.ndiff(template, content)))
        def error(msg, i):
            diff.insert(i+2, "! <<< Parsing failed before reaching here >>>\n")
            logging.info('\n' + ''.join(diff))
            logging.error(msg)
            raise ValueError(f"'{filepath}' does not match the template: '{filename}'")
        i = 0
        while i < len(diff):
            if "- MULTILINE_PLACEHOLDER" in diff[i]:  # don't use exact match to avoid cases without trailing newline
                i += 1
                while i < len(diff) and diff[i].startswith('+ '):
                    i += 1
                continue
            if i+1 >= len(diff):
                error("Odd lines", i)
            if not diff[i].startswith('- ') or not diff[i+1].startswith('+ '):
                error("Expected no line deletion and addition", i)
            regexp = "^" + re.escape(diff[i][2:]).replace('PLACEHOLDER', '.*') + "$"
            if not re.match(regexp, diff[i+1][2:]):
                error("Expected line deletion and addition to differ only in the placeholder", i)
            i += 2

compare_file_with_template(".devcontainer/devcontainer.json", ignored_workspaces=["ros1_bridge_ws"])
compare_file_with_template(".gitignore")
compare_file_with_template("docker/.bashrc")
compare_file_with_template("docker/.dockerignore", ignored_workspaces=["ros1_bridge_ws", "orbslam3_ws"])
