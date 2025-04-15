FROM ros:humble

# 安装依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install flask

# 创建工作目录
WORKDIR /ros2_ws

# 复制 ROS2 包
COPY . /ros2_ws/src/ros2_driver/

# 构建 ROS2 包
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-select ros2_driver

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/ros2_driver/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/ros2_ws/install/ros2_driver/lib:$LD_LIBRARY_PATH

# 设置入口点
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /ros2_ws/install/setup.bash && exec \"$@\""]
CMD ["ros2", "launch", "ros2_driver", "driver.launch.py"] 