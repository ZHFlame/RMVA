FROM ros:humble-ros-base

# 基础依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    libopencv-dev \
    libeigen3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 裁判系统 SDK 及依赖
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /rmva/src
COPY . /rmva/src/challenge

WORKDIR /rmva
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select challenge

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /rmva/install/setup.bash && bash"]