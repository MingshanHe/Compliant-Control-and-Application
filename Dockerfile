# ROS 2 Lyrical + Gazebo Sim environment for the simulation branch (sim-ros2).
#
#   docker build -t compliant-control:sim .
#   docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix compliant-control:sim
#   ros2 launch robot_sim_bringup sim.launch.py robot:=ur5
FROM ros:lyrical

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-vcstool python3-colcon-common-extensions ros-lyrical-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/Compliant-Control-and-Application
COPY . .
RUN if [ ! -d robots/franka/franka_description ]; then vcs import robots < robots/robots.repos; fi

RUN apt-get update && rosdep update \
    && rosdep install --from-paths /ws/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN source /opt/ros/lyrical/setup.bash && colcon build --symlink-install
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
