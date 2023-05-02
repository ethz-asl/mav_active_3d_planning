FROM ros:noetic
RUN apt-get install ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-geographic-msgs autoconf libyaml-cpp-dev protobuf-compiler libgoogle-glog-dev liblapacke-dev libgeographic-dev
pip install future unrealcv
RUN mkdir /ws/src/mav_active_3d_planning
WORKDIR /ws
RUN catkin init \
    && catkin config --extend /opt/ros/melodic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel
WORKDIR /ws/src
COPY mav_active_3d_planning_https.rosinstall /ws/src/dependencies.rosinstall
RUN wstool update
COPY . mav_active_3d_planning
RUN catkin build .


