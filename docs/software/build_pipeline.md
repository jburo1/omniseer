# Build Pipeline

This page gives the high-level view of how software changes move through the
repository.

## Current Pipeline Layers

Omniseer currently has two main build/validation layers:

- local developer workflows used during active development
- GitHub Actions workflows used for shared validation and docs publishing

## Local Development Flow

Most code changes are still validated locally first.

Typical examples:

- `colcon build` and `colcon test` for the ROS workspace
- targeted CMake builds for native vision components
- local linting such as Ruff

This keeps iteration fast and makes it easier to debug issues close to the
change.

## Shared Validation Flow

After code is pushed or opened in a pull request, GitHub Actions runs the
repository's CI workflow to validate the main supported package set.

See [CI/CD Overview](ci_cd.md) for:

- workflow triggers
- exact checks that run in CI
- what is currently deployed automatically
- current gaps and future expansion points

## Future Direction

As the project grows, this page can become the umbrella view for:

- local build workflows
- CI validation stages
- release packaging
- artifact publishing
- hardware-in-the-loop execution

For now, the detailed operational reference lives in
[CI/CD Overview](ci_cd.md).
