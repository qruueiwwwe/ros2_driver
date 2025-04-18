# 使用ROS2 Humble作为基础镜像
FROM ros:humble-ros-base

# 设置工作目录
WORKDIR /ros2_ws

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 复制项目文件
COPY src/ros2_driver /ros2_ws/src/ros2_driver

# 安装Python依赖
RUN pip3 install -e /ros2_ws/src/ros2_driver

# 构建ROS2工作空间
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/lib/python3.10/site-packages:$PYTHONPATH

# 设置入口点
ENTRYPOINT ["ros2", "launch", "ros2_driver", "driver.launch.py"]
