# AGENTS.md

Guidance for AI coding agents working in this repository.

## User Preferences

- Be concise, direct, and action-oriented.
- Prefer doing the work over describing the work. Inspect first, then implement, verify, and summarize.
- Make reasonable assumptions instead of asking unnecessary questions. If a decision has real tradeoffs, present a short recommendation and a small set of options.
- Keep progress updates brief and informative.
- Keep final summaries high-signal. Avoid long file-by-file changelogs unless explicitly requested.
- Prefer fixes that persist across sessions and machines. When possible, encode behavior in repository files instead of one-off local shell state.
- Match the repository's recent commit style: conventional prefix with an explicit scope, followed by a descriptive sentence. Mention notable secondary changes when helpful.

## Repository Conventions

- Preserve existing architecture and naming patterns in `vision/`, `ros_ws/src/`, `.devcontainer/`, and related support tooling.
- Favor Doxygen-style C++ comments to stay aligned with the documented code style.
- When adding or changing ROS 2 message definitions, update both `package.xml` and `CMakeLists.txt` for the affected package.
- Prefer focused diffs and targeted validation over broad refactors.
- Do not commit generated artifacts, machine-specific secrets, or host-only paths unless the change explicitly requires them.

## Workflow Defaults

- Start by gathering context with targeted searches, diffs, and file reads.
- Before editing files, briefly state what is being changed.
- After changes, report what changed, what was verified, and any remaining limitation.
- If hardware, container, or SBC-specific constraints block full verification, say so clearly instead of implying success.

## Environment Notes

- This repository is often used from an SSH-connected SBC and from a devcontainer.
- If Git pushes fail in a devcontainer because the wrong forwarded GitHub identity is selected, prefer a repo-local `git config core.sshCommand` fix over changing the remote URL globally.
- Prefer repository-local or clone-local persistence when it helps future sessions without polluting the host machine.
