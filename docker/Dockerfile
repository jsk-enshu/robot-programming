FROM ros:melodic-ros-core-bionic

RUN apt -y update \
    && apt install -y \
    ros-melodic-desktop-full \
    iputils-ping \
    net-tools \
    wget \
    bash \
    fluxbox \
    git \
    net-tools \
    novnc \
    supervisor \
    x11vnc \
    xterm \
    xvfb \
    iproute2 \
    python-wstool \
    python-catkin-tools

RUN apt -y update \
    && apt install -y \
    python-rosdep

RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash"
RUN echo "source /opt/ros/melodic/setup.sh" >> /root/.bashrc
WORKDIR /root/catkin_ws
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes \
    ROS_MASTER_URI=http://localhost:11311
COPY . /app
CMD ["/app/entrypoint.sh"]
EXPOSE 8080
