# Robot Runtime Container

The robot runtime container packages the supported real-hardware ROS workspace into
an image that starts through the same `scripts/omni run real` surface used on the
host. The robot-side image carries the real operator profile, native RKNN vision
bridge, gateway process, telemetry recording tools, and hardware IO launch wiring.
It does not include simulation, firmware flashing, docs, or laptop-only
development tooling.

The final image is based on `ros:kilted-ros-core-noble`, builds the workspace in a
separate stage, and copies only the merged ROS install tree, source-built
micro-ROS agent, runtime scripts, RKNN runtime library, and direct runtime apt
dependencies into the final image. Build-only tools such as `gcc`, `g++`,
`colcon`, and `protoc` are absent from the final image. `cmake` remains installed
because the Kilted ROS runtime packages currently depend on it.

## Build

On the ROCK 5B+ image, build the robot runtime image:

```bash
scripts/omni build runtime-container
```

For checkpointed robot-local publishing, prefer the runtime workflow wrapper:

```bash
scripts/omni runtime build
```

This builds
`ghcr.io/jburo1/omniseer-robot-runtime:robot-candidate-<UTC>-g<shortsha>` by
default and records local build metadata under `.omniseer/runtime/`. The
`g<shortsha>` suffix is the first 12 characters of the source git commit. Override
the image base with `--image <base>` or `OMNISEER_RUNTIME_IMAGE`; pass `--tag`
only when you need an explicit custom checkpoint tag.

The robot runtime image requires RKNN SDK files on the build host:

- `rknn_api.h`, default `/usr/include/rknn_api.h`
- `librknnrt.so`, auto-detected through `ldconfig`

The build wrapper enables Docker BuildKit so those files can be passed as temporary
named build contexts. They are copied into the image, but they are not added to the
repository build context.

The Kilted apt repositories used on the ROCK 5B+ image do not publish
`ros-kilted-micro-ros-agent`, so the image builds the upstream
`micro-ROS-Agent` Kilted source package, pinned by default to commit
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

For non-robot validation, build the portable image target. It excludes
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

Runtime builds embed the source revision in `OMNISEER_GIT_SHA` and the
`org.opencontainers.image.revision` label. For traceable container experiments,
also pass image and experiment metadata through environment variables or matching
`scripts/omni run real` record flags:

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

The generated run report surfaces the recorded source revision, image reference,
image digest, launch command/profile/arguments, experiment config, experiment
parameters, and the latest platform snapshots from `manifest.yaml` and
`system.jsonl`.

Use the portable image only for launch and entrypoint checks; it defaults to
`start_vision:=false` and is not a full robot runtime.

## Checkpoint and Promotion

The robot-local promotion loop is:

```bash
git checkout <passed-master-commit>
scripts/omni runtime build
scripts/omni runtime record
scripts/omni runtime verify
scripts/omni runtime verify --stage full
scripts/omni runtime push
```

For a full pre-registry verification pass on the robot:

```bash
git checkout <passed-master-commit>
sudo scripts/omni runtime build
sudo scripts/omni runtime record

RUN_DIR="$(ls -dt runs/operator_* | head -n 1)"
scripts/omni runs inspect "${RUN_DIR}"
scripts/omni runs report "${RUN_DIR}" --overwrite

sudo scripts/omni runtime verify
sudo scripts/omni runtime verify --stage full
sudo scripts/omni runtime push
```

Use the exact `master` commit whose GitHub Actions `ci` run passed. Review the
generated run report before `runtime push`. The push step is the promotion boundary:
it refuses to publish unless the git working tree is clean, full verification passed
for the same local image ID, and the verification metadata commit matches the
current checkout.

`runtime record` is the manual operator evidence-capture path. It uses the latest
local runtime build by default, starts `run real --profile operator ... bringup`
with automatic Docker TTY detection, writes the runbundle to `runs/operator_<UTC>`
on the host, and samples system telemetry every second. Stop manual runs with
Ctrl-C or the monitor GUI Stop Run button when the experiment is complete;
autonomy runs exit when target centering reaches a terminal state. Graceful
shutdown lets the recorder finalize `manifest.yaml` and `summary.json`. The
monitor GUI also issues `scripts/omni runtime stop --run-id <id>` immediately
for runtime-backed runs; that command targets the named runtime record container
for the run id, confirms it is stopped or already absent, and lets the GUI become
ready for the next run without waiting on a stale SSH session.

When launched from a devcontainer, the runtime wrapper resolves the host-side
workspace bind path before starting Docker so the runtime container writes back
into this checkout's `runs/` directory instead of a separate `/omniseer/runs`
directory on the Docker host. Override that detection with
`OMNISEER_RUNTIME_RUNS_HOST_ROOT=/host/path/to/omniseer/runs` if needed.

`runtime verify` defaults to a safe smoke stage. It starts the container with
vision, Teensy, LiDAR, boundary-topic waits, and pre-launch cleanup disabled; a
launch that survives until the smoke timeout is treated as a pass. Full
verification runs the container's existing real operator smoke path with run
recording enabled. Verification runs Docker without an interactive TTY so it can
be used under `sudo`, SSH automation, and other non-interactive launch paths.
Direct `runtime run` and manual `runtime record` commands allocate a TTY only
when attached to one by default; override that with
`OMNISEER_RUNTIME_DOCKER_TTY=always` or `never`.

`runtime push` promotes the verified local image to immutable
`robot-verified-g<full-commit-sha>` and moving `robot-verified` tags. Pass
`--release-tag <tag>` to publish one additional release label for the same verified
image. Candidate build tags remain local unless you explicitly use one as the source
tag for this promotion flow. The push records release metadata under
`.omniseer/runtime/`, including the commit SHA, local image ID, registry digest,
pushed tags, and full-verification metadata path.

Pull the latest verified image on the robot with:

```bash
scripts/omni runtime pull
```

Use an immutable tag for rollback or pinning:

```bash
scripts/omni runtime pull --tag robot-verified-g437c109075310102030405060708090a0b0c0d0e
```

For recorded robot experiments, prefer the registry digest recorded after push. If
that digest is unavailable, record the full-commit immutable tag. Avoid using only
the moving `robot-verified` tag as experiment provenance.

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
| `omniseer/robot-runtime:v2` | 2.84 GB | Runtime image with source-built micro-ROS Agent |
| devcontainer image | 4.99 GB | Full development environment |

The runtime image is smaller than the devcontainer and does not carry the
compiler, colcon, or protobuf compiler in the final image. The image size is
mostly from the Python OpenCV stack,
FFmpeg/GStreamer plugins, controller-manager and control message dependencies,
source-built micro-ROS Agent dependencies, and broad ROS runtime dependencies
pulled in by Kilted packages. Some large
development-labeled packages still arrive transitively through runtime package
dependencies rather than from the explicit builder stage.

Do not remove runtime packages only to shrink the image. First measure image
history and package ownership, then remove a dependency only when the robot
runtime verification path still passes and the removed package is not part of the
operator, hardware IO, native vision, gateway, or recording surface.
