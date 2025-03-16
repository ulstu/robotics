FROM ros:noetic

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3-teleop \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-bringup \
    git \
    mc \
    htop \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-vision-opencv \
    ros-noetic-rviz \
    ros-noetic-image-transport-plugins \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    python3-opencv \
    libopencv-dev \
    ros-noetic-image-proc \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /root/catkin_ws && \
    mkdir /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git 

RUN echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc  && \
    echo "export LIBGL_ALWAYS_INDIRECT=0" >> /root/.bashrc  && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc

    # Установка рабочего каталога
WORKDIR /root/catkin_ws

# Запуск bash для ROS
ENTRYPOINT ["/bin/bash"]
