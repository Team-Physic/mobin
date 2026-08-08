FROM osrf/ros:jazzy-desktop-full

ARG TB3_SIM_COMMIT=unknown
ARG CALIB_COMMIT=unknown
ARG RERUN_VERSION=0.35.0

LABEL lab.turtlebot3_simulations.commit="${TB3_SIM_COMMIT}"
LABEL lab.direct_visual_lidar_calibration.commit="${CALIB_COMMIT}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-dev-tools \
    mesa-utils \
    libgl1-mesa-dri \
    python3-venv \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}"

ENV PATH="/opt/rerun/bin:${PATH}"

ENV TURTLEBOT3_MODEL=waffle_pi

WORKDIR /ws

CMD ["bash"]
