# Code Documentation Guide

This repository already uses MkDocs (see `mkdocs.yml`) for narrative/project documentation. This guide standardizes *code-level* documentation across:

- C/C++ (`vision/`, C++ ROS packages under `ros_ws/src/**`, firmware C/C++ under `firmware/`)
- Python (ROS 2 nodes/launch under `ros_ws/src/**`)
- ROS interfaces (`.msg`, `.srv`, `.action`) and configuration (`*.yaml`, `*.launch.py`)

The goal is that a new contributor can answer “what does this do?”, “how do I use it?”, and “what can go wrong?” directly from the source and generated API docs.

---

## What “documented” means here

Document at three levels:

1. **Package / module docs**: a short `README.md` in each ROS package directory and/or a MkDocs page under `docs/`.
2. **Public API docs**: Doxygen comments for C/C++ headers in `**/include/**`, and docstrings for public Python modules/classes/functions.
3. **Operational docs**: ROS nodes’ parameters/topics/services/actions, plus any non-obvious runtime assumptions (frames, units, QoS, timing, hardware).

As a rule of thumb: if someone can call/use it without reading the `.cpp` body, it needs documentation.

---

## C/C++ documentation (Doxygen)

### Standard: Doxygen blocks on public API

Use Doxygen-style comments for:

- Any header installed/exposed via `**/include/**`
- Public classes/structs/enums and public methods
- “Hot path” or safety-critical code (DMA-BUF ownership, threading, ISR constraints, real-time loops)

Recommended style (already used in `ros_ws/src/omniseer_description/include/omniseer/frontier.hpp`):

- File header: `\file` + short description
- Entities: `\brief` for the one-liner, `\details` for the long form
- Parameters/returns: `\param`, `\return`
- Errors: `\throws` (or document error signaling if exceptions are not used)
- Invariants/ownership: `\note` / `\warning`

Example (adapt to your symbol names):

```cpp
/**
 * \brief Capture NV12 frames from a V4L2 device via a kernel buffer ring.
 * \details
 * - Ownership: frames are borrowed from the driver ring via `dequeue()` and must be returned with
 *   `requeue(out.v4l2_index)` to avoid stalling capture.
 * - Threading: not thread-safe; call all methods from the same thread.
 */
class V4l2Capture { /* ... */ };
```

### Use groups to keep APIs navigable

For libraries that form a coherent module, define a group once and attach public types/functions:

```cpp
/** \defgroup omniseer_vision Vision
 *  \brief Vision capture + preprocessing pipeline.
 */
```

Then, in headers:

```cpp
/** \ingroup omniseer_vision */
```

### Document the “contract”, not the implementation

For each public method, capture:

- **Preconditions** (e.g., “must call `start()` before `dequeue()`”)
- **Postconditions** (what state changes, what gets returned/filled)
- **Performance / real-time constraints** (blocking, timeouts, allocations)
- **Ownership / lifetime** (e.g., DMA-BUF fd validity, requeue requirements)
- **Units and frames** (meters vs cells, nanoseconds, map vs odom frame, pixel format)

This is especially important for the vision DMA-BUF pipeline (see `vision/include/omniseer/vision/types.hpp` and `vision/include/omniseer/vision/v4l2_capture.hpp`), where the most valuable documentation is the *buffer lifetime and requeue contract*.

### Prefer documenting in headers

- Put full API documentation in the header (`.hpp/.h`) where the API is declared.
- Use `.cpp` comments for tricky implementation details only.

### Add a Doxygen config (Doxyfile)

This repo includes a root `Doxyfile` for generating C/C++ API docs. There is also an example Doxygen configuration in `ros_ws/src/yolo_ros/.github/Doxyfile` (vendored upstream).

Minimum recommended settings (illustrative, adjust paths as needed):

```ini
PROJECT_NAME           = "omniseer"
OUTPUT_DIRECTORY       = build/docs/doxygen
RECURSIVE              = YES
INPUT                  = vision/include vision/src ros_ws/src firmware
FILE_PATTERNS          = *.h *.hpp *.c *.cpp *.md
EXCLUDE_PATTERNS       = */build/* */install/* */log/* */.venv/* */ros_ws/src/yolo_ros/*
WARN_IF_UNDOCUMENTED   = YES
EXTRACT_ALL            = NO
GENERATE_HTML          = YES
```

If you later want to integrate C++ API docs more tightly with other doc systems, enable `GENERATE_XML = YES` as well.

---

## Python documentation (ROS 2)

### Standard: Google-style docstrings

Keep using module/class docstrings (already present in `ros_ws/src/analysis/analysis/path_recorder.py`). Add docstrings where the behavior isn’t obvious:

- Node classes: purpose + parameters + topics/services/actions + QoS expectations
- Entry points: `main()` behavior and exit conditions
- Non-trivial helpers: inputs/outputs/units

Suggested pattern for a node class:

```py
class PathRecorder(Node):
    """Cache and replay odom and simulation paths.

    Parameters:
        odom_topic (str): Odometry topic (default: `/odometry/filtered`).
        sim_topic (str): Simulation odom topic (default: `/gz_odom`).
        max_path_length (int): Max poses kept per path.

    Publishes:
        /odom_path (nav_msgs/msg/Path): Recorded filtered odom path.
        /sim_path (nav_msgs/msg/Path): Recorded simulation path.
    """
```

If you want docstrings to be enforced automatically, enable Ruff’s `D` rules (pydocstyle) in `pyproject.toml`.

---

## ROS interfaces and configuration

### `.msg` / `.srv` / `.action`

Document:

- What the message represents
- Units, frame ids, coordinate conventions
- Any special values (sentinels, NaNs, “0 means unknown”, etc.)

ROS IDL files support line comments; use them above fields and at the top of the file.

### `*.launch.py`, `*.yaml`

Add a short header comment describing:

- What the launch/config is for (sim vs real, perception vs navigation)
- Required external resources (hardware, Gazebo world, topic remaps)
- The main parameters a user is expected to touch

---

## Generating API documentation

### Option A (recommended): Doxygen for C/C++ API docs

1. Add a root `Doxyfile` (or `docs/Doxyfile`) that scans:
   - `vision/include`, `vision/src`
   - `ros_ws/src/**/include` and relevant `ros_ws/src/**/src`
   - firmware code under `firmware/` (if you want it included)
2. Exclude generated/build trees:
   - `**/build/**`, `**/install/**`, `**/log/**`, and vendor directories
3. Generate HTML locally into a build directory (recommended), e.g. `build/docs/doxygen/html`.

Minimal commands:

- Generate docs: `doxygen Doxyfile`
- View docs: open `build/docs/doxygen/html/index.html`

### Local MkDocs preview

The docs site uses MkDocs Material (see `mkdocs.yml` and `.github/workflows/docs.yml`).

- Install: `pip install mkdocs-material mkdocs-git-revision-date-localized-plugin`
- Serve locally: `mkdocs serve`

### Option B: Python API docs in MkDocs (optional)

If you want rendered Python API docs from docstrings inside MkDocs, add `mkdocstrings` + `mkdocs-gen-files` (or a similar plugin) and generate pages for packages under `ros_ws/src/`.

---

## Publishing docs (GitHub Pages)

This repo’s `.github/workflows/docs.yml` deploys MkDocs when `docs/**` or `mkdocs.yml` change.

If you want the API docs to be published together with the MkDocs site:

1. Generate Doxygen HTML during the workflow (install `doxygen` + `graphviz`).
2. Copy the generated HTML into the MkDocs build input (or into `site/` before deploy).
3. Add a link from a MkDocs page (e.g., `docs/software/code_documentation.md`) to the generated `api/cpp/index.html`.

Avoid committing generated HTML to `main`—generate it in CI instead.

### Publishing MkDocs + Doxygen together (recipe)

One pragmatic approach is:

1. Run `doxygen Doxyfile` into `build/docs/doxygen/html`.
2. Build MkDocs into `site/` via `mkdocs build`.
3. Copy `build/docs/doxygen/html` into `site/api/cpp/`.
4. Deploy with `mkdocs gh-deploy --no-build`.

In GitHub Actions, that means:

- Add dependencies: `sudo apt-get update && sudo apt-get install -y doxygen graphviz`
- Replace the final step in `.github/workflows/docs.yml` with the sequence above
- Expand the workflow `paths:` so doc deploys run when code changes (e.g., `vision/**`, `ros_ws/src/**`, `firmware/**`, and `Doxyfile`)

---

## Package README template (ROS 2)

For each package under `ros_ws/src/<pkg>/`, add a `README.md` that includes:

- Purpose (1–2 paragraphs)
- Nodes (name, what it does)
- Parameters (name, type, default, meaning)
- Topics/services/actions (name, type, direction)
- Launch examples (exact `ros2 launch ...` commands)
- Known limitations / assumptions (frames, units, QoS, hardware)

## Rollout checklist (practical)

Use this sequence to “document the whole project” without boiling the ocean:

1. **Add package READMEs** for each ROS package in `ros_ws/src/*/README.md` (purpose + nodes + parameters + launch examples).
2. **Document public headers** first (`**/include/**`), starting with the highest-impact APIs:
   - Vision types and capture/processing pipeline
   - Navigation/frontier selection API
3. **Document ROS nodes** (Python/C++): parameters/topics/services/actions/QoS.
4. **Document interfaces** (`omniseer_msgs`, `yolo_msgs`): top-of-file + field/unit comments.
5. **Add generation** (Doxygen) and (optionally) **publish** in CI.

Automation suggestions (optional but high-leverage):

- **Doxygen warnings as a gate**: set `WARN_IF_UNDOCUMENTED = YES` (and optionally `WARN_AS_ERROR = YES`) so missing docs are visible early.
- **Docstrings as a gate**: enable Ruff `D` rules if you want docstrings required on public Python APIs.
- **Keep formatting automatic**: run `pre-commit run -a` before opening PRs (clang-format + ruff are already configured).

“Definition of done” for a documented component:

- Public API is described (what/why/how, units, ownership, errors)
- Example usage exists (snippet or launch example)
- Any assumptions are explicit (threading, real-time, hardware, frame conventions)
