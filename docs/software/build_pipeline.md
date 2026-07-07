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
repository's CI workflow in several focused lanes:

- `lint`
- `ros-core`
- `bringup-smoke`
- `vision-host`
- `firmware-build`
- `docs-build`

This keeps the default PR gate broad enough to catch interface drift while
keeping the slower checks isolated instead of hiding inside one large job.

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

The active product direction adds two planned layers after shared validation:

- creation of a reproducible perception experiment bundle
- provider-neutral synchronization and hosted review of completed runs

Neither layer is implemented today. The current pipeline stops at validated source,
build/test artifacts, and GitHub Pages documentation deployment.
