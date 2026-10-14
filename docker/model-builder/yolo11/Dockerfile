# Host-only, linux/amd64. Export and compile share Toolkit2-compatible dependencies.
FROM python:3.10.16-slim-bookworm@sha256:f9fd9a142c9e3bc54d906053b756eb7e7e386ee1cf784d82c251cf640c502512
ENV PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONDONTWRITEBYTECODE=1 \
    YOLO_CONFIG_DIR=/tmp/ultralytics MPLCONFIGDIR=/tmp/matplotlib \
    YOLO_AUTOINSTALL=false
RUN apt-get update && apt-get install --no-install-recommends -y \
    ca-certificates git libglib2.0-0 libgl1 && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/airockchip/ultralytics_yolo11.git /opt/ultralytics-yolo11 \
    && git -C /opt/ultralytics-yolo11 checkout --detach 0692e9297670acf4cc6d0cec773d7a9493cb8a5f
COPY requirements.txt /opt/yolo11-requirements.txt
RUN python -m pip install --no-cache-dir pip==24.3.1 setuptools==75.6.0 wheel==0.45.1 \
    && python -m pip install --no-cache-dir --no-deps torch==2.2.0+cpu torchvision==0.17.0+cpu \
       --index-url https://download.pytorch.org/whl/cpu \
    && python -m pip install --no-cache-dir -r /opt/yolo11-requirements.txt \
    && python -m pip install --no-cache-dir --no-deps --no-build-isolation /opt/ultralytics-yolo11 \
    && python -m pip check \
    && python -c 'import ultralytics; assert ultralytics.__version__ == "8.3.9"; from rknn.api import RKNN'
# Build-time import caches must not become root-owned runtime caches.
RUN rm -rf /tmp/ultralytics /tmp/matplotlib
WORKDIR /workspace
