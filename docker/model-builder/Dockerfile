# Host-only YOLO-World v2 exporter and RKNN compiler. This image is never
# used by the robot runtime image.
# This is the linux/amd64 child manifest rather than the multi-architecture
# index: legacy Docker builders otherwise can select an incompatible child.
FROM python:3.8.18-slim-bullseye@sha256:803387b3f7ddc94cceb15ad0929ea85ee9d21a01e245414417bac1fc7547a2a3

ARG YOLO_WORLD_REPOSITORY=https://github.com/airockchip/YOLO-World.git
ARG YOLO_WORLD_REVISION=b8b0fe9beffa9564306a798f6e443c9fe88057af
ARG RKNN_TOOLKIT2_REVISION=deaba85fc437a28db0b0c29f27d8929f4c5816a1

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1

RUN apt-get update \
    && apt-get install --no-install-recommends -y ca-certificates curl git libglib2.0-0 libgl1 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules "${YOLO_WORLD_REPOSITORY}" /opt/yolo-world \
    && git -C /opt/yolo-world checkout --detach "${YOLO_WORLD_REVISION}" \
    && git -C /opt/yolo-world submodule update --init --recursive \
    && curl -fsSL "https://raw.githubusercontent.com/airockchip/rknn-toolkit2/${RKNN_TOOLKIT2_REVISION}/rknn-toolkit2/packages/requirements_cp38-2.1.0.txt" \
      -o /tmp/rknn-requirements.txt \
    && curl -fsSL "https://raw.githubusercontent.com/airockchip/rknn-toolkit2/${RKNN_TOOLKIT2_REVISION}/rknn-toolkit2/packages/rknn_toolkit2-2.1.0+708089d1-cp38-cp38-linux_x86_64.whl" \
      -o /tmp/rknn_toolkit2-2.1.0+708089d1-cp38-cp38-linux_x86_64.whl

# These versions are compatible with Rockchip's v2.1.0 Toolkit2 cp38 wheel
# and the pinned YOLO-World exporter. Install the Toolkit requirements first
# because they constrain ONNX and protobuf versions used by RKNN conversion.
RUN python -m pip install --no-cache-dir --upgrade pip==23.3.2 setuptools==68.2.2 wheel==0.41.3 \
    && python -m pip install --no-cache-dir \
      torch==2.0.0+cpu torchvision==0.15.1+cpu \
      --index-url https://download.pytorch.org/whl/cpu \
    && python -m pip install --no-cache-dir \
      -r /tmp/rknn-requirements.txt \
    && python -m pip install --no-cache-dir \
      /tmp/rknn_toolkit2-2.1.0+708089d1-cp38-cp38-linux_x86_64.whl \
    && python -m pip install --no-cache-dir \
      https://download.openmmlab.com/mmcv/dist/cpu/torch2.0.0/mmcv-2.0.0-cp38-cp38-manylinux1_x86_64.whl \
    && python -m pip install --no-cache-dir \
      mmengine==0.10.3 mmdet==3.0.0 mmyolo==0.6.0 \
      timm==0.6.13 transformers==4.36.2 tokenizers==0.15.2 \
      opencv-python-headless==4.9.0.80

RUN python -c 'import onnx; from rknn.api import RKNN'

WORKDIR /workspace
