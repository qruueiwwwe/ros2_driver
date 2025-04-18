FROM ros:humble

# 安装必要的依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install flask pyyaml opencv-python numpy

# 创建工作空间
WORKDIR /ros2_ws/src
COPY . /ros2_ws/src/

# 编译工作空间
WORKDIR /ros2_ws

# 复制 ROS2 包
COPY . /ros2_ws/src/ros2_driver/

# 确保 Python 文件有执行权限
RUN chmod +x /ros2_ws/src/ros2_driver/ros2_driver/*.py

# 构建 ROS2 包
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-select ros2_driver

# 手动创建 libexec 目录和可执行文件
RUN mkdir -p /ros2_ws/install/ros2_driver/lib/ros2_driver && \
    ln -sf /ros2_ws/src/ros2_driver/ros2_driver/driver_node.py /ros2_ws/install/ros2_driver/lib/ros2_driver/driver_node && \
    ln -sf /ros2_ws/src/ros2_driver/ros2_driver/cmd_vel_relay_node.py /ros2_ws/install/ros2_driver/lib/ros2_driver/cmd_vel_relay_node && \
    chmod +x /ros2_ws/install/ros2_driver/lib/ros2_driver/*

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/ros2_driver/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/ros2_ws/install/ros2_driver/lib:$LD_LIBRARY_PATH

# 设置入口点
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /ros2_ws/install/setup.bash && exec \"$@\""]

# 修改 CMD 命令
CMD ["ros2", "launch", "ros2_driver", "driver.launch.py"]

