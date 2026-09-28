# Explorer Diagrams

Create the top-level architecture explorer here:

```text
docs/diagrams/explorer/system-explorer.d2
```

Start the file by importing the shared classes:

```d2
...@../shared/classes

direction: down
```

The first explorer should show only the navigational architecture root:

```text
PC
Simulation
RKNN model build
Operator application
Robot boundary
Robot runtime
Firmware
Physical hardware
RunBundle
Static report
```

Keep labels at the top level generic:

```text
commands
telemetry / video
control
sensor feedback
evidence
review
```

Do not include ROS topic names, device paths, CI infrastructure, or implementation
details in this diagram. Implementation-level container details are normally out
of scope, but the host-side RKNN model-build environment may be shown as an
architectural execution/deployment boundary. Keep its label at that level (for
example, `RKNN model build`) and do not expose Docker internals.

Show deployment of the RK3588 `.rknn` model artifact from the development PC to
the robot as an offline path. It is distinct from the operator application's live
commands, telemetry, and evidence workflow, and must not imply that robot
inference depends on the development PC at runtime.

Internal links should resolve from the rendered SVG path:

```text
docs/assets/diagrams/explorer/system-explorer.svg
```

Example link targets:

```d2
link: "../../../verification/evidence/"
link: "../../../architecture/overview/"
link: "../../../operations/operator-run-workflow/"
```

Render target:

```text
docs/assets/diagrams/explorer/system-explorer.svg
```
