# Robot Runtime Container

_Status: v2 build path implemented; target hardware validation still required_

The robot runtime container packages the supported real-hardware ROS workspace into
an image that starts through the same `scripts/omni run real` surface used on the
host. The v2 target is robot-side only: it carries the real operator profile, native
RKNN vision bridge, gateway process, telemetry recording tools, and hardware IO
launch wiring. It does not include simulation, firmware flashing, docs, or
laptop-only development tooling.

V2 makes the runtime boundary stricter than v1. The final image is based on
`ros:kilted-ros-core-noble`, builds the workspace in a separate stage, and copies
only the merged ROS install tree, source-built micro-ROS agent, runtime scripts,
RKNN runtime library, and direct runtime apt dependencies into the final image.
Build-only tools such as `gcc`, `g++`, `colcon`, and `protoc` are absent from the
final v2 image. `cmake` remains installed because the Kilted ROS runtime packages
currently depend on it.

## Build

On the ROCK 5B+ image, build the robot target:

```bash
scripts/omni build runtime-container
```

The robot target requires RKNN SDK files on the build host:

- `rknn_api.h`, default `/usr/include/rknn_api.h`
- `librknnrt.so`, auto-detected through `ldconfig`

The build wrapper enables Docker BuildKit so those files can be passed as temporary
named build contexts. They are copied into the image, but they are not added to the
repository build context.

As of the current Kilted apt repositories used on the ROCK 5B+ image,
`ros-kilted-micro-ros-agent` is not published. V2 builds the upstream
`micro-ROS-Agent` Kilted source package instead, pinned by default to commit
`f0d809138de19a61fe1a640d5c5a3076cd648360`. The source build installs the
`micro_ros_agent` ROS package and Micro XRCE-DDS Agent runtime library into the
merged `/ros_ws/install` tree.

Override those paths when the SDK is installed elsewhere:

```bash
scripts/omni build runtime-container \
  --rknn-include /path/to/rknn_api.h \
  --rknn-lib /path/to/librknnrt.so
```

Override the source commit only when intentionally testing a newer upstream agent:

```bash
scripts/omni build runtime-container \
  --micro-ros-agent-ref <git-ref>
```

For non-robot validation, build the portable target. It excludes
`omniseer_vision_bridge`, so it can prove the container build and entrypoint without
RKNN/RGA hardware SDK files:

```bash
scripts/omni build runtime-container \
  --target portable-runtime \
  --image omniseer/robot-runtime:portable
```

## Run

The container needs host networking and direct hardware access for ROS discovery,
serial devices, camera nodes, DMA heaps, RGA, and RKNN runtime paths:

```bash
docker run --rm -it \
  --privileged \
  --network=host \
  --pid=host \
  --ipc=host \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  -v "$PWD/runs:/runs" \
  omniseer/robot-runtime:v2
```

Additional launch arguments pass through to `scripts/omni`:

```bash
docker run --rm -it --privileged --network=host -v /dev:/dev -v "$PWD/runs:/runs" omniseer/robot-runtime:v2 \
  run real --profile operator --record-run container_smoke --record-out /runs/container_smoke bringup
```

For traceable container experiments, pass image and experiment metadata through
environment variables or matching `scripts/omni run real` record flags:

```bash
docker run --rm -it --privileged --network=host \
  -e OMNISEER_CONTAINER_IMAGE_REF=omniseer/robot-runtime:v2 \
  -e OMNISEER_CONTAINER_IMAGE_DIGEST=sha256:<digest> \
  -e OMNISEER_EXPERIMENT_CONFIG=experiments/container-smoke.yaml \
  -e OMNISEER_EXPERIMENT_PARAMETERS=profile=operator,scenario=container_smoke \
  -v /dev:/dev \
  -v "$PWD/runs:/runs" \
  omniseer/robot-runtime:v2 \
  run real --profile operator --record-run container_smoke --record-out /runs/container_smoke bringup
```

Use the portable image only for launch and entrypoint checks; it defaults to
`start_vision:=false` and is not a full robot runtime.

## Verification Boundary

Local image builds verify that the ROS install tree is baked into the image and that
the entrypoint sources `/opt/ros/$ROS_DISTRO` and `/ros_ws/install`. Full validation
still requires the robot: camera capture, DMA heap access, RGA preprocessing, RKNN
inference, Teensy/micro-ROS, LiDAR, and preview streaming all cross hardware or
kernel-driver boundaries.

## Size Boundary

Measured locally on the ROCK 5B+ build host:

| Image | Size | Notes |
| --- | ---: | --- |
| `omniseer/robot-runtime:v2` | 2.84 GB | Current runtime target with source-built micro-ROS Agent |
| `omniseer/robot-runtime:v1` | 3.5 GB | Initial bringup image |
| devcontainer image | 4.99 GB | Full development environment |
| `ros:kilted-ros-core-noble` | 768 MB | V2 base image |
| `ros:kilted-ros-base-noble` | 1.33 GB | V1 base image |

V2 is acceptable as a bringup/runtime artifact because it is smaller than the
devcontainer and no longer carries the compiler, colcon, or protobuf compiler in
the final image. It is not yet a lean production runtime. The remaining size is
mostly from the Python OpenCV stack, FFmpeg/GStreamer plugins, controller-manager
and control message dependencies, source-built micro-ROS Agent dependencies, and
broad ROS runtime dependencies pulled in by Kilted packages. Some large
development-labeled packages still arrive transitively through runtime package
dependencies rather than from the explicit builder stage.
