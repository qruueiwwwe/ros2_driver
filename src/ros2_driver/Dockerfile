FROM ros:humble-ros-base

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install flask

COPY ./src ./src

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

ENV PYTHONPATH=/ros2_ws/install/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV LD_LIBRARY_PATH=/ros2_ws/install/lib:/opt/ros/humble/lib
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/fastrtps_profile.xml

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch ros2_driver driver.launch.py"]
