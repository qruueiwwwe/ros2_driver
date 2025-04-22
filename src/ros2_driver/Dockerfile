# 使用ROS2 Humble作为基础镜像
FROM ros:humble

# 设置工作目录
WORKDIR /ros2_ws

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install flask

# 复制工作空间文件
COPY . /ros2_ws/src/ros2_driver/

# 确保Python脚本有执行权限
RUN chmod +x /ros2_ws/src/ros2_driver/ros2_driver/*.py

# 构建ROS2工作空间
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --symlink-install

# 设置环境变量
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/ros2_ws/install/lib/python3.10/site-packages
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/ros2_ws/install/lib
ENV ROS_ROOT=/opt/ros/humble/share/ros
ENV ROS_PACKAGE_PATH=/opt/ros/humble/share:/ros2_ws/src
ENV ROS_DISTRO=humble

# 设置入口点和默认命令
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch ros2_driver driver.launch.py"] 