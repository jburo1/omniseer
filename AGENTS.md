# AGENTS.md

Instructions for Codex agents working in this repository.

## Mission

This repository exists to ship a robust embodied AI system end to end:

`sense -> decide -> act -> observe -> evaluate -> redeploy`

Optimize for:

- correctness on real hardware
- stable interfaces
- reproducible verification
- low-latency and low-overhead execution
- clean, maintainable code

Do not optimize for:

- cleverness
- framework churn
- abstraction for its own sake
- broad rewrites without clear operational value
- "agent impressive" patches that increase complexity without clear payoff

## Default Loop

The default loop is:

`inspect -> implement -> verify -> summarize`

Prefer the smallest change that fully solves the problem.

## Decision Rules

- Apply Occam's razor: prefer the simplest explanation and the smallest effective fix.
- Prefer targeted fixes over broad refactors.
- Fix root causes when they are near and clear. Avoid speculative cleanup.
- Make reasonable low-risk assumptions and continue.
- Ask only when the decision is materially ambiguous, expensive to reverse, or likely to create user-visible churn.

## Architecture Boundaries

- `robot-core` is sacred. Mission-critical robot behavior must not depend on optional operator tooling.
- If preview, diagnostics, dashboard, or teleop tooling fails, robot-core must continue to operate safely.
- Keep the mission path, gateway/control plane, operator UI, and infra/tooling as separate concerns unless the task explicitly requires crossing those boundaries.
- Treat the robot, gateway, UI, and CI as one product with clear ownership boundaries.
- Preserve existing architecture unless the task clearly requires structural change.

General ownership rule:

- robot-core owns robot behavior
- gateway owns external contracts and orchestration
- ui/operator owns presentation and operator workflows
- infra/tooling owns reproducibility, packaging, deployment, and automation

## Contracts and Interfaces

- Prefer explicit control/status contracts over exposing internal ROS topic or service names.
- Keep protobuf, gRPC, ROS messages, and other inter-process contracts stable.
- Evolve contracts additively where possible.
- Do not casually rename, repurpose, or overload fields.
- Reserve or otherwise preserve removed protobuf field numbers when applicable.
- When changing ROS or gateway interfaces, keep definitions, generated code, package metadata, and build wiring in sync.

## Verification Standard

- Return evidence, not confidence.
- A change is not done because it looks right. It is done when the behavior is specified, verified, and the result is reported clearly.
- Run the smallest meaningful validation first.
- Prefer targeted tests, builds, lint, and runtime checks over broad expensive sweeps.
- Do not claim success without verification.
- If hardware, camera, ROS graph, container, or SBC constraints block full validation, say so explicitly.

For bug fixes:

- reproduce first if practical
- add a regression test if practical
- prove the failure is gone

For hot paths or timing-sensitive code:

- mention expected latency, throughput, memory, CPU, or startup impact when relevant
- avoid "should be fine" language
- measure if the change plausibly affects timing or memory

## Safety and Operational Rules

Never silently change:

- control semantics
- state machine semantics
- operator-visible statuses
- API meanings
- timing-sensitive behavior
- startup or shutdown behavior
- fault handling behavior

If such a change is necessary:

- call it out explicitly
- update tests and docs
- describe compatibility or migration implications

Optional systems should degrade gracefully when possible and fail loudly when necessary.

Preserve or improve logs, telemetry, and failure reporting when changing runtime behavior.

## Working Style

- Read surrounding code, tests, and adjacent docs before editing.
- Match the local structure and style of the code you touch.
- Keep diffs focused and reviewable.
- Prefer durable repository changes over one-off shell state.
- Assume the repo may be used from an SBC, over SSH, or in a devcontainer; avoid host-specific assumptions.
- Do not revert unrelated changes in the worktree.
- Do not commit generated artifacts, secrets, or host-only paths unless the task explicitly requires them.
- Update the nearest relevant docs when you change an operational surface, contract, or lifecycle-critical behavior.
- Prefer boring foundations that compound: stable scripts, explicit contracts, replayability, and observability.

## Local Conventions Observed in This Repo

Match surrounding code first. Current repository patterns include:

- C++ uses `clang-format` with LLVM-based style, Allman braces, 2-space indentation, and ~100 column lines.
- C++ types and enums use `PascalCase`.
- C++ functions, methods, free helpers, locals, and parameters use `snake_case`.
- Private C++ members typically use a leading underscore, for example `_preview_manager`.
- C++ code favors explicit ownership, simple control flow, early returns, `const auto` for obvious temporaries, and `std::unique_ptr` over shared ownership unless sharing is required.
- C++ translation units often keep narrow helper conversions in an unnamed namespace instead of introducing extra abstraction.
- Python is `ruff` formatted with 4-space indentation, double quotes, and a 120-column limit.
- Python code favors small modules, small functions, explicit argument parsing, and type hints where they improve clarity.
- Proto and spec docs carry semantic comments that explain behavior, versioning rules, and invariants, not just field names.
- Long-form docs tend to be concise engineering specs: purpose, current state, design considerations, non-goals, and rollout status.
- External APIs prefer bounded enums and explicit messages over free-form strings.
- Repository code often favors explicit normalized state snapshots and narrow adapters over generic passthrough layers.

## Performance Bias

This repo targets edge, robotics, and real-time-ish systems.

Do not introduce changes that:

- add unnecessary copies on hot paths
- block mission-critical loops without justification
- increase control-loop jitter
- create uncontrolled memory growth
- add avoidable startup latency
- move critical work onto less deterministic paths without a clear reason

## Dependency Policy

Do not add a dependency unless it clearly improves one of:

- correctness
- maintainability
- verification
- performance
- delivery speed

When proposing or adding one:

- justify it briefly
- prefer mature, boring dependencies
- avoid duplicating capabilities already present in the repo

## Communication

- Be concise, direct, and technical.
- Keep progress updates short, factual, and useful.
- Final summaries should cover what changed, what was verified, and what remains uncertain.
- If presenting options, recommend one first and explain tradeoffs briefly.

## Commit Style

- When asked to commit, use a conventional prefix with an explicit scope and a descriptive sentence.
- Mention notable secondary changes when they materially help review.
