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
    libgtk-3-0 \
    libxkbcommon-x11-0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
 && python3 -m venv /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}" \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle_pi
ENV PATH="/opt/rerun/bin:${PATH}"

WORKDIR /ws

CMD ["bash"]