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

After code is pushed to `master`, GitHub Actions runs one or more focused
path-filtered workflows:

- `ci` for normal portable software changes
- `runtime` for portable runtime container packaging changes
- `firmware` for firmware and micro-ROS workspace changes
- `docs` for documentation changes and `gh-pages` publishing

This keeps the default branch checks easy to reason about: software changes test
software, packaging changes test the container, firmware changes test firmware,
and documentation changes build and publish documentation.

See [CI/CD Overview](ci-cd.md) for:

- workflow triggers
- exact checks that run in CI
- what is currently deployed automatically
- coverage boundaries

## Pipeline Scope

This page is the umbrella view for:

- local build workflows
- CI validation stages
- release packaging
- documentation publishing
- hardware-in-the-loop execution

The detailed operational reference lives in [CI/CD Overview](ci-cd.md). Robot-run
artifacts are produced by the RunBundle workflow rather than by CI.
