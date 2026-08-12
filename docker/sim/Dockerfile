FROM osrf/ros:jazzy-desktop-full

ARG TB3_SIM_COMMIT=unknown
ARG CALIB_COMMIT=unknown

LABEL lab.turtlebot3_simulations.commit="${TB3_SIM_COMMIT}"
LABEL lab.direct_visual_lidar_calibration.commit="${CALIB_COMMIT}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-turtlebot3 \
    ros-dev-tools \
    mesa-utils \
    libgl1-mesa-dri \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle_pi

WORKDIR /ws

CMD ["bash"]