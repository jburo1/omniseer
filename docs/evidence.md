## CI verified

- Ruff is pinned to `0.13.3` in `pyproject.toml`, `.pre-commit-config.yaml`, and `.github/workflows/ci.yml`.

## hardware verified

- Not verified for this change; it only updates lint/tooling metadata and docs.

## planned

- Reviewers can run `pre-commit run -a`, the local Ruff command from `docs/software/ci_cd.md`, and the existing docs build.
