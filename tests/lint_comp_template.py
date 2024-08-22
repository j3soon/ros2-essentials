import difflib
import glob
import logging
import os
import re
from pathlib import Path

logging.basicConfig(level=logging.INFO)
current_dir = os.path.dirname(os.path.realpath(__file__))
repo_dir = os.path.realpath(f"{current_dir}/..")

def get_regexp_from_template_line(line):
    regexp = re.escape(line) \
        .replace(r'\{PLACEHOLDER_\#\}', '(# )?') \
        .replace(r'\{PLACEHOLDER\}', '.*')
    return f"^{regexp}$"

def compare_file_with_template(filepath, targetpath=None, ignored_workspaces=[]):
    targetpath = targetpath or filepath
    logging.info(f"Checking if '{targetpath}' matches the template...")
    template_path = f"{repo_dir}/tests/diff_base/{filepath}"
    template = Path(template_path).read_text().splitlines(keepends=True)  # keepends to preserve trailing newlines
    for filename in glob.glob(f"{repo_dir}/*_ws/{targetpath}"):
        # Skip certain cases intentionally
        if any(ws in filename for ws in ignored_workspaces):
            continue
        logging.debug(f"Checking: '{filename[len(repo_dir)+1:]}'...")
        content = Path(filename).read_text().splitlines(keepends=True)
        diff = list(filter(lambda x: x.startswith('- ') or x.startswith('+ ') or x.startswith('  '), difflib.ndiff(template, content)))
        def error(msg, i):
            diff.insert(i+2, "! <<< Parsing failed before reaching here >>>\n")
            logging.info('\n' + ''.join(diff))
            logging.error(msg)
            raise ValueError(f"'{filepath}' does not match the template: '{filename}'")
        i = 0
        while i < len(diff):
            if "- {PLACEHOLDER_MULTILINE}" in diff[i]:  # don't use exact match to avoid cases without trailing newline
                i += 1
                while i < len(diff) and diff[i].startswith('+ '):
                    i += 1
                continue
            if diff[i].startswith('  '):
                i += 1
                continue
            if i+1 >= len(diff):
                error("Odd lines", i)
            # Stack and compare
            if diff[i].startswith('- '):
                stack = [diff[i]]
                j = i+1
                while j < len(diff) and diff[j].startswith('- '):
                    stack.append(j)
                    j += 1
                k = 0
                while k < len(stack) and j+k < len(diff) and diff[j+k].startswith('+ '):
                    if "- {PLACEHOLDER_MULTILINE}" in diff[i+k]:
                        raise error("{PLACEHOLDER_MULTILINE} is not supported in this case yet", i)
                    regexp = get_regexp_from_template_line(diff[i+k][2:])
                    if not re.match(regexp, diff[j+k][2:]):
                        error("Expected line deletion and addition to differ only in the placeholder", i)
                    k += 1
                if k != len(stack):
                    error("Expected no line deletion and addition", i)
            elif diff[i].startswith('+ '):
                stack = [diff[i]]
                j = i+1
                while j < len(diff) and diff[j].startswith('+ '):
                    stack.append(j)
                    j += 1
                k = 0
                while k < len(stack) and j+k < len(diff) and diff[j+k].startswith('- '):
                    if "- {PLACEHOLDER_MULTILINE}" in diff[j+k]:
                        raise error("{PLACEHOLDER_MULTILINE} is not supported in this case yet", i)
                    regexp = get_regexp_from_template_line(diff[j+k][2:])
                    if not re.match(regexp, diff[i+k][2:]):
                        error("Expected line deletion and addition to differ only in the placeholder", i)
                    k += 1
                if k != len(stack):
                    error("Expected no line deletion and addition", i)
            else:
                raise ValueError("Unexpected diff prefix")
            i = j + k

compare_file_with_template(".devcontainer/devcontainer.json", ignored_workspaces=["ros1_bridge_ws"])
compare_file_with_template(".gitignore")
compare_file_with_template("docker/.bashrc")
compare_file_with_template("docker/.dockerignore", ignored_workspaces=["ros1_bridge_ws", "orbslam3_ws"])
compare_file_with_template("docker/compose.yaml", ignored_workspaces=["ros1_bridge_ws"])
compare_file_with_template("docker/Dockerfile", ignored_workspaces=["ros1_bridge_ws"])
