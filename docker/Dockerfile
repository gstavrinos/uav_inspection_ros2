FROM ros:galactic-ros-core

LABEL maintainer="George Stavrinos <gstavrinos@iit.demokritos.gr>"

RUN export DEBIAN_FRONTEND=noninteractive && apt-get update &&\
apt-get install -y \
git \
xvfb \
curl \
gnupg \
gradle \
python3-pip \
lsb-release \
python3-empy \
libgmock-dev \
python3-rosdep \
openssh-client \
openssh-server \
ros-galactic-rqt* \
ros-galactic-desktop \
ros-galactic-eigen3-cmake-module \
python3-colcon-common-extensions &&\
/bin/bash -c "source /opt/ros/galactic/setup.bash" &&\
rosdep init &&\
rosdep update &&\
pip3 install -U pyros-genmsg setuptools &&\
echo "alias rosdep_install='rosdep install -i --from-path src --rosdistro galactic -y'" >> ~/.bashrc &&\
echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc &&\
echo ". ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc &&\
echo ". /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc &&\
echo "export _colcon_cd_root=~/ros2_ws" >> ~/.bashrc &&\
/bin/bash -c "source /opt/ros/galactic/setup.bash" &&\
mkdir -p ~/ros2_ws/src &&\
mkdir ~/sources &&\
git clone https://github.com/gstavrinos/uav_inspection_ros2 ~/ros2_ws/src/uav_inspections_ros2 &&\
git clone https://github.com/PX4/px4_ros_com ~/ros2_ws/src/px4_ros_com &&\
git clone https://github.com/PX4/px4_msgs ~/ros2_ws/src/px4_msgs &&\
git clone https://github.com/eProsima/foonathan_memory_vendor.git ~/sources/foonathan_memory_vendor &&\
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.0 ~/sources/FastDDS-2.0.0 &&\
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/sources/Fast-RTPS-Gen &&\
/bin/bash -c "source ~/.bashrc" &&\
cd ~/sources/foonathan_memory_vendor &&\
mkdir build &&\
cd build &&\
cmake .. &&\
cmake --build . --target install &&\
cd ~/sources/FastDDS-2.0.0 &&\
mkdir build &&\
cd build &&\
cmake -DTHIRDPARTY=ON -DSECURITY=ON .. &&\
make -j$(nproc --all) &&\
make install &&\
cd ~/sources/Fast-RTPS-Gen &&\
./gradlew assemble &&\
./gradlew install &&\
cd ~/ros2_ws/src/px4_ros_com/scripts &&\
bash build_ros2_workspace.bash
