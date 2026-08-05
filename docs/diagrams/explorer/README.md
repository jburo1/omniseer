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
Operator
Laptop tooling
Robot boundary
Robot mission runtime
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

Do not include ROS topic names, containers, device paths, CI infrastructure, or
implementation details in this diagram.

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
