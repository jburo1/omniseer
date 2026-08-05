# Omniseer Architecture Diagrams

This directory contains source diagrams for the Omniseer documentation. D2 source
files live under `docs/diagrams/`; rendered SVGs live under `docs/assets/diagrams/`.
MkDocs excludes the D2 sources from published site output.

Rendered diagram artifacts belong under `docs/assets/diagrams/`.

## Initial Explorer Contract

The first diagram is the top-level navigational root for the architecture docs. It
should answer:

> What are the major parts of Omniseer, where do they run, and how do they
> communicate?

It should not expose ROS topic names, containers, device paths, CI infrastructure,
or implementation-level details.

## Source And Artifact Mapping

```text
docs/diagrams/shared/classes.d2
docs/diagrams/explorer/system-explorer.d2
  -> docs/assets/diagrams/explorer/system-explorer.svg
```

## Render Contract

Use a pinned D2 renderer and explicit layout so SVG diffs are intentional.

```text
Supported D2 version: v0.7.1
Layout engine: dagre
Theme: 0
```

Expected render shape:

```bash
d2 --layout dagre --theme 0 \
  docs/diagrams/explorer/system-explorer.d2 \
  docs/assets/diagrams/explorer/system-explorer.svg
```

Use the repository front door for normal operation:

```bash
scripts/omni docs diagrams
scripts/omni docs diagrams --check
```

The docs build runs the complete local verification pipeline:

```bash
scripts/omni docs build
```

That command checks committed SVG freshness, performs a clean strict MkDocs build,
and validates internal links embedded in built SVG diagrams.

## Explorer Nodes

| Node | Meaning at this level | Initial destination |
| --- | --- | --- |
| Operator | Human initiating runs and reviewing results. | `../../../operations/operator-run-workflow/` |
| Laptop tooling | Off-robot control, diagnostics, preview, retrieval, and report workflow. | `../../../operations/operator-run-workflow/` |
| Robot mission runtime | Native perception plus ROS 2 autonomy, recording, and integration. | `../../../architecture/overview/` |
| Firmware | Teensy-owned low-level motor and sensor integration over micro-ROS. | `../../../architecture/overview/` |
| Physical hardware | Electrical and physical sensors, actuators, compute, and power system. | `../../../architecture/overview/` |
| RunBundle | Durable robot-run evidence artifact. | `../../../verification/evidence/` |
| Static report | Human-readable derived review artifact. | `../../../verification/evidence/` |

Use "Robot mission runtime" at the top level rather than "ROS 2 mission runtime"
because the mission software includes native V4L2/RGA/RKNN perception as well as
ROS 2 integration.

## Visual Vocabulary

Node classes:

```text
boundary        Runtime or location boundary.
operator        Human actor.
laptop          Off-robot operator tooling.
robot_runtime   Mission software on robot compute.
firmware        MCU control boundary.
hardware        Physical and electrical system.
evidence        Durable artifact or report.
optional        Non-mission or supporting path.
```

Edge classes:

```text
command_edge    Operator or control command flow.
telemetry_edge  Telemetry, video, or status flow.
evidence_edge   Recording, retrieval, or report flow.
offline_edge    Offline or post-run flow.
```

The encoding should stay orthogonal:

```text
Node fill       Ownership or execution boundary.
Node shape      Component versus durable artifact.
Border style    Required versus optional.
Edge color      Command, telemetry, or evidence.
Edge dash       Live versus offline flow.
```

## Link Convention

The published SVG is embedded from:

```text
docs/assets/diagrams/explorer/system-explorer.svg
```

Links inside the SVG resolve relative to the SVG location in the built site, not
relative to the Markdown page embedding it. Internal D2 links should therefore use
built-site-relative paths such as:

```text
../../../verification/evidence/
../../../architecture/overview/
../../../operations/operator-run-workflow/
```

The docs build validates SVG links after a clean MkDocs build so stale `site/`
output cannot hide broken diagram navigation.

## Verification Target

The completed explorer milestone should satisfy:

- the explorer communicates the operating and evidence loop without surrounding
  prose;
- every clickable node resolves to an authoritative documentation page;
- D2 source is kept under `docs/diagrams/` and excluded from `site/`;
- the committed SVG is reproducible with the pinned D2 version and explicit
  layout;
- `scripts/omni docs diagrams --check` detects stale, missing, and orphaned
  artifacts without modifying the working tree;
- `scripts/omni docs build` checks SVG freshness, runs a clean strict MkDocs
  build, and validates built-site SVG links.
