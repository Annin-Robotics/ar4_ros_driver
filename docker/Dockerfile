FROM ros:jazzy

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-ros-gz

COPY . /src
RUN rosdep install --from-paths /src --ignore-src -r -y && \
    rm -rf /src

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
    # echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc