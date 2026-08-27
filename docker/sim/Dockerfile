FROM osrf/ros:jazzy-desktop-full

ARG TB3_SIM_COMMIT=unknown
ARG CALIB_COMMIT=unknown
ARG RERUN_VERSION=0.35.0
ARG ULTRALYTICS_VERSION=8.3.220

ENV PIP_DEFAULT_TIMEOUT=1000
ENV PIP_RETRIES=10

LABEL lab.turtlebot3_simulations.commit="${TB3_SIM_COMMIT}"
LABEL lab.direct_visual_lidar_calibration.commit="${CALIB_COMMIT}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-jazzy-cv-bridge \
    ros-jazzy-message-filters \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-twist-mux \
    ros-jazzy-vision-msgs \
    libeigen3-dev \
    libyaml-cpp-dev \
    ros-dev-tools \
    mesa-utils \
    libgl1-mesa-dri \
    python3-venv \
    libgtk-3-0 \
    libxkbcommon-x11-0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/cu121 \
    "torch==2.5.1" "torchvision==0.20.1"

RUN python3 -m pip install --no-cache-dir --break-system-packages \
    "numpy<2" \
    "opencv-python==4.11.0.86" \
    "ultralytics==${ULTRALYTICS_VERSION}" \
    https://github.com/ultralytics/CLIP/archive/refs/heads/master.zip

RUN python3 -m venv /opt/rerun \
 && /opt/rerun/bin/pip install --no-cache-dir "rerun-sdk==${RERUN_VERSION}" \
 && ln -s /opt/rerun/bin/rerun /usr/local/bin/rerun \
 && rosdep update

ENV TURTLEBOT3_MODEL=waffle_pi

WORKDIR /ws

CMD ["bash"]
