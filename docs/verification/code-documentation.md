# Code Documentation Guide

This page describes what code-level documentation should contain in Omniseer.
Keep comments close to the code they explain, and focus on behavior that a
contributor or operator cannot safely infer from names alone.

Document public surfaces enough that someone can answer:

- what does this do?
- what inputs, outputs, units, frames, and lifetimes does it use?
- what can block, fail, allocate, or affect robot timing?
- what topics, parameters, services, actions, or files does it expose?

## C/C++ Interfaces

Use Doxygen-style comments for public C/C++ APIs, especially headers under
`**/include/**`, firmware interfaces, and hot-path vision or robot-control code.

Document the contract rather than restating the implementation:

- preconditions and postconditions
- ownership and lifetime, such as DMA-BUF file descriptors or V4L2 requeue
  requirements
- threading expectations and callback context
- blocking behavior, timeouts, allocations, and expected error reporting
- units, coordinate frames, pixel formats, and timestamp semantics
- safety or performance constraints that affect robot behavior

Prefer full API documentation in headers where the API is declared. Use `.cpp`
comments for non-obvious implementation details, invariants, or hardware quirks.

Example:

```cpp
/**
 * \brief Capture NV12 frames from a V4L2 device via a kernel buffer ring.
 * \details
 * - Ownership: dequeued frames must be returned with `requeue(out.v4l2_index)`.
 * - Threading: call all methods from the same thread.
 */
class V4l2Capture { /* ... */ };
```

## Python Interfaces

Use concise docstrings for public Python modules, classes, functions, ROS nodes,
and entry points when behavior is not obvious from the name.

For ROS nodes, document:

- purpose and lifecycle
- parameters, defaults, units, and valid ranges
- published and subscribed topics with message types
- services or actions
- QoS expectations when they matter
- exit conditions and failure behavior

Example:

```py
class PathRecorder(Node):
    """Cache and publish odometry paths.

    Parameters:
        odom_topic: Odometry topic to record.
        max_path_length: Maximum poses retained.

    Publishes:
        /odom_path: Recorded odometry path.
    """
```

## ROS Interfaces

For `.msg`, `.srv`, and `.action` files, use line comments to document:

- what the interface represents
- units and coordinate frames
- timestamp meaning
- special values, sentinels, NaNs, or empty strings
- compatibility notes when fields are added or preserved

Do not repurpose fields without documenting the migration implication.

## Launch And Configuration

For launch files and YAML configuration, add short comments when the file has
non-obvious operational constraints.

Document:

- whether the file is for sim, real hardware, or shared graph composition
- required devices, models, parameter files, or external processes
- parameters operators are expected to override
- topic remaps, frame assumptions, and QoS choices
- safety, startup, shutdown, timeout, or watchdog behavior

Comments should describe the implemented system. Avoid roadmap notes and
aspirational documentation work in source-adjacent files.
