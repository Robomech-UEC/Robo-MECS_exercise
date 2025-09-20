FROM tiryoh/ros2-desktop-vnc:humble

RUN apt-get update \
    && apt-get install -y sudo \
    && apt-get install -y vim && apt-get install -y nano \
    && apt-get install libeigen3-dev \
    && rm -rf /var/lib/apt/lists

RUN apt-get update \
    && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

RUN cd /home/ubuntu/Documents \
    && git clone -b main https://github.com/Robomech-UEC/Robo-MECS.git \
    && git clone -b main https://github.com/Robomech-UEC/Robo-MECS_exercise.git\
