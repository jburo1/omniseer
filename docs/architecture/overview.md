# System Architecture

## Interactive System Map

<object data="../../assets/diagrams/explorer/system-explorer.svg" type="image/svg+xml" aria-label="Omniseer system explorer" width="100%"></object>

Click a component to open its detailed documentation.

## Overview

The development PC builds and prepares models; the robot runs the deployed
runtime and records the primary evidence; the operator laptop starts and
observes runs, then retrieves artifacts for review. The operational flow is
model build → on-robot runtime → RunBundle → laptop review and static report.

The onboard boundary includes camera capture, RGA preprocessing, RKNN
inference, ROS 2 contracts, bounded behavior, command arbitration, firmware
I/O, and run recording. Operator controls, preview, retrieved bundles, and
reports sit beyond that boundary: they support operation and review, but are
not dependencies of mission-critical perception or control.

The evidence/review boundary preserves the recorded RunBundle and its raw
artifacts for inspection; reports and annotations are derived review surfaces.
For representative results, see [v2-L FP Target Acquisition](../verification/target-acquisition.md),
[Six-Model Detector Comparison](../verification/detector-comparison.md), and
[v2-L INT8 Quantization Failure Analysis](../verification/int8-quantization-failure.md).
