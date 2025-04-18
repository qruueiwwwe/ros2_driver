FROM ros:humble

# 安装必要的依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
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
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select ros2_driver

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/ros2_driver/lib/python3.10/site-packages:$PYTHONPATH
ENV LD_LIBRARY_PATH=/ros2_ws/install/ros2_driver/lib:$LD_LIBRARY_PATH
ENV ROS_DOMAIN_ID=0
ENV PATH=/opt/ros/humble/bin:$PATH

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros2_driver", "driver.launch.py"]

