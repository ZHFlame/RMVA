FROM vision-vrena-2025:v0.1.2

# 复制现有构建产物、选手包及结果
COPY ./install /home/va/Vision-Vrena-2025/install
COPY ./src/player_pkg /home/va/Vision-Vrena-2025/src/player_pkg
COPY ./results /home/va/Vision-Vrena-2025/results

# 进入工作空间并安装额外依赖
WORKDIR /home/va/Vision-Vrena-2025
RUN apt-get update && \
    apt-get upgrade -y && \
    mkdir -p src/referee_pkg/results && \
    apt-get install -y \
        ros-humble-xacro \
        ros-humble-gazebo-ros-pkgs && \
    rm -rf /var/lib/apt/lists/*


RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select player_pkg

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]