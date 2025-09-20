FROM tiryoh/ros2-desktop-vnc:humble

RUN apt-get update \
    && apt-get install -y sudo \
    && apt-get install -y vim && apt-get install -y nano \
    && apt-get install -y libeigen3-dev \
    && rm -rf /var/lib/apt/lists

RUN apt-get update \
    && apt-get install -y ros-humble-ros2-control \
    && apt-get install -y ros-humble-ros2-controllers \
    && apt-get install -y ros-humble-gazebo-ros2-control \
    && apt-get install -y ros-humble-joint-state-publisher \
    && apt-get install -y ros-humble-joint-state-publisher-gui

RUN mkdir -p /home/ubuntu/Robomech && cd /home/ubuntu/Robomech \
    && git clone -b main https://github.com/Robomech-UEC/Robo-MECS.git \
    && git clone -b main https://github.com/Robomech-UEC/Robo-MECS_exercise.git
