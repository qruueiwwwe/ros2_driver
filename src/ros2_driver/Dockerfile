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


# 复制项目文件
COPY . /ros2_ws/src/ros2_driver

# 构建ROS2工作空间
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/ros2_ws/install/lib:$LD_LIBRARY_PATH
ENV ROS_PACKAGE_PATH=/opt/ros/humble/share:/ros2_ws/src:$ROS_PACKAGE_PATH
ENV ROS_MASTER_URI=http://192.168.31.118:11311
ENV PATH=/opt/ros/humble/bin:$PATH

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros2_driver", "driver.launch.py"]
