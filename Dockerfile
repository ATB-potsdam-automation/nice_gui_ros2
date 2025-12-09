ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Install general dependencies
RUN apt update && apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp ros-${ROS_DISTRO}-rmw-zenoh-cpp python3-pip python3-opencv

# Pip dependencies
RUN python3 -m pip install --break-system-packages nicegui geopandas

WORKDIR /workspace
RUN rosdep update

# Install app package
COPY . /workspace/src/
RUN apt update && rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Add source to bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT [ "/ros_entrypoint.sh" ]