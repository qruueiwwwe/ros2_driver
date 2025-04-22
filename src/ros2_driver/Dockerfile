# 使用ROS2 Humble作为基础镜像
FROM ros:humble-ros-base

# 设置工作目录
WORKDIR /ros2_ws

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-flask \
    python3-opencv \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS2 依赖包
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2 \
    && rm -rf /var/lib/apt/lists/*

# 复制项目文件到容器中
COPY src /ros2_ws/src

# 安装 Python 依赖
RUN pip3 install -r /ros2_ws/src/ros2_driver/requirements.txt

# 构建 ROS2 工作空间
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# 设置环境变量，使 ROS2 可以找到工作空间
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 设置启动命令
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch ros2_driver driver.launch.py"]
