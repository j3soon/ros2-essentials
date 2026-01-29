# Contribution Guide

We welcome all kinds of contributions, including but not limited to bug reports, documentation improvements, new features, and bug fixes. Please ensure to always open an issue to discuss the changes you want to make before sending a pull request.

## Overview

1. Ensure there is an [opened GitHub issue](https://github.com/j3soon/ros2-essentials/issues) corresponding to the changes you want to make.
2. Self-assign the GitHub issue.
3. Fork the repository by visiting [ros2-essentials](https://github.com/j3soon/ros2-essentials) and clicking the "Fork" button.
4. Clone the forked repository to your local machine, e.g., `git clone git@github.com:username/ros2-essentials.git`
5. Create a new branch for your changes, e.g., `git checkout -b feat/your-feature-name` for a feature branch or `git checkout -b fix/your-bug-name` for a bug fix branch.
6. Make your changes and ensure they are tested. To ensure reproducibility, consider re-clone your pushed branch and rerun the `post_install.sh` script to clear all untracked files and cached data.
7. Push your local changes to your forked repository, e.g., `git push origin feat/your-feature-name`.
8. Create a pull request to merge your changes into the main repository's `main` branch.

## Guidelines

1. Commit messages should follow the [conventional commit](https://www.conventionalcommits.org/en/v1.0.0/) format.
2. Commit description should include as much details as possible. Try to include references you consulted and your rationale behind the changes. This will be essential for reviewers to understand the changes and for future reference. (e.g., [`058ff59`](https://github.com/j3soon/ros2-essentials/commit/058ff598ae705c95516353cd14a71945176c337e))
3. If copied files from other sources, please include the source in the commit message. If copied from GitHub, include the permalink (link with commit ID) and the branch name. (e.g., [`6a2dac1`](https://github.com/j3soon/ros2-essentials/commit/6a2dac186e1321dc7f7ec961ad22628bffc352d3))
4. Each commit should be atomic and self-contained.
5. When updating `template_ws` defaults, follow up with a separate unify workspace style commit (preferred message: `feat: Unify workspaces style`) that applies the minimal template-alignment changes across all workspaces. Keep this separate from the template update commit.
6. If anyone helped you during the development, please include them in the commit message using the `Co-authored-by` format (e.g., [`4f9832d`](https://github.com/j3soon/ros2-essentials/commit/4f9832d7a0c4156449d95b3503f9928d2d7de376) due to [this comment](https://github.com/j3soon/ros2-essentials/pull/85#discussion_r2204978905)). These include, but are not limited to, feedback from reviewers, in-person discussions, and pair programming sessions.
7. Refrain from force-pushes after a reviewer has started reviewing your changes. The only exception is for new contributors when a reviewer has explicitly asked you to rebase your branch or modify commit messages.
8. Use USDA instead of USD file format whenever possible for better readability and version control. (e.g., [PR#101](https://github.com/j3soon/ros2-essentials/pull/101))

Use `git log` and search for related commits to understand the commit message format.

## Scripts

When creating a new workspace, use the `scripts/create_workspace.sh <workspace_name>` script to set up the workspace structure.

## Linting

Basic linting is done by running the following script:

```sh
tests/test_all.sh
```

## Pull Request References

- Creating a new module, see [PR#93](https://github.com/j3soon/ros2-essentials/pull/93) and [PR#95](https://github.com/j3soon/ros2-essentials/pull/95).
- Creating and updating a workspace, see [PR#83](https://github.com/j3soon/ros2-essentials/pull/83), [PR#85](https://github.com/j3soon/ros2-essentials/pull/85), and [PR#101](https://github.com/j3soon/ros2-essentials/pull/101).
- Updating Isaac Sim/Lab, see [PR#92](https://github.com/j3soon/ros2-essentials/pull/92).
- Using OmniGraph and USDA, see [PR#83](https://github.com/j3soon/ros2-essentials/pull/83).
