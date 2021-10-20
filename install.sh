#!/bin/bash
sudo apt-get update && sudo apt-get install git curl gnupg lsb-release libgmock-dev python3-pip -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-galactic-desktop ros-galactic-rqt* ros-galactic-eigen3-cmake-module ros-galactic-joint-state-publisher clang-format-10 python3-setuptools python3-vcstool python3-colcon-common-extensions python3-flake8 python3-pytest-cov python3-rosdep python3-empy python-numpy dmidecode libgstreamer1.0-0 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-libav libeigen3-dev libgazebo11-dev libgstreamer-plugins-base1.0-dev libimage-exiftool-perl libopencv-dev libxml2-utils pkg-config protobuf-compiler gradle ros-galactic-gazebo-ros-pkgs ros-galactic-moveit-core ros-galactic-moveit-common ros-galactic-moveit-plugins -y
sudo pip3 install -U pyros-genmsg setuptools jinja2 argcomplete argparse cerberus coverage matplotlib numpy nunavut packaging pandas pkgconfig pygments wheel pymavlink pyserial pyulog requests six toml flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
. /opt/ros/galactic/setup.bash
sudo rosdep init
rosdep update
echo "alias rosdep_install='rosdep install -i --from-path src --rosdistro galactic -y'" >> ~/.bashrc
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "source ~/px4_ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "source ~/moveit2_ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_ws" >> ~/.bashrc
. /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
mkdir -p ~/px4_ros2_ws/src
mkdir -p ~/moveit2_ros2_ws/src
mkdir ~/sources
cd ~/ros2_ws/src
sudo rm -rf ~/ros2_ws/src/uav_inspections_ros2
sudo rm -rf ~/ros2_ws/src/odom_to_tf_ros2
sudo rm -rf ~/ros2_ws/src/gazebo_ros2_control
sudo rm -rf ~/ros2_ws/src/moveit_visual_tools
sudo rm -rf ~/ros2_ws/build
sudo rm -rf ~/ros2_ws/install
sudo rm -rf ~/px4_ros2_ws/src/px4_ros_com
sudo rm -rf ~/px4_ros2_ws/src/px4_msgs
sudo rm -rf ~/px4_ros2_ws/build
sudo rm -rf ~/px4_ros2_ws/install
sudo rm -rf ~/moveit2_ros2_ws/src/*
sudo rm -rf ~/moveit2_ros2_ws/build
sudo rm -rf ~/moveit2_ros2_ws/install
sudo rm -rf ~/sources/foonathan_memory_vendor
sudo rm -rf ~/sources/PX4-Autopilot
sudo rm -rf ~/sources/Fast-RTPS-Gen
sudo rm -rf ~/sources/FastDDS-2.0.0
# git clone https://github.com/gstavrinos/uav_inspection_ros2 ~/ros2_ws/src/uav_inspections_ros2
git clone https://github.com/gstavrinos/odom_to_tf_ros2 ~/ros2_ws/src/odom_to_tf_ros2
git clone https://github.com/ros-simulation/gazebo_ros2_control ~/ros2_ws/src/gazebo_ros2_control
git clone -b ros2 https://github.com/ros-planning/moveit_visual_tools ~/ros2_ws/src/moveit_visual_tools
git clone https://github.com/PX4/px4_ros_com ~/px4_ros2_ws/src/px4_ros_com
# git clone https://github.com/ros-planning/moveit2 -b 2.2.1 ~/moveit2_ros2_ws/src/moveit2
git clone https://github.com/ros-planning/moveit2 ~/moveit2_ros2_ws/src/moveit2
git clone https://github.com/ros-controls/ros2_control ~/moveit2_ros2_ws/src/ros2_control
git clone https://github.com/ros-controls/ros2_controllers ~/moveit2_ros2_ws/src/ros2_controllers
git clone https://github.com/PX4/px4_msgs ~/px4_ros2_ws/src/px4_msgs
git clone https://github.com/eProsima/foonathan_memory_vendor.git ~/sources/foonathan_memory_vendor
git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.12.0 ~/sources/PX4-Autopilot
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.0 ~/sources/FastDDS-2.0.0
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/sources/Fast-RTPS-Gen
. ~/.bashrc
cd ~/sources/foonathan_memory_vendor
mkdir build
cd build
cmake ..
sudo cmake --build . --target install
cd ~/sources/FastDDS-2.0.0
mkdir build
cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
cd ~/sources/Fast-RTPS-Gen
./gradlew assemble
sudo ./gradlew install
cd ~/px4_ros2_ws/src/px4_ros_com/scripts
sudo bash build_ros2_workspace.bash
. ~/px4_ros2_ws/install/local_setup.sh
. ~/.bashrc
cd ~/sources/PX4-Autopilot
DONT_RUN=1 make px4_sitl_rtps gazebo
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro galactic -y
colcon build --symlink-install
cd ~/moveit2_ros2_ws/src/moveit2
git checkout 5ff36ac4855411a8eb8eac636d00ef3838d9611e
cd ~/moveit2_ros2_ws/src/ros2_control
git checkout 017e3c5053ef268ad907df2fc252ebcde390b05b
cd ~/moveit2_ros2_ws/src/ros2_controllers
git checkout 79ee39d944d2adaa30253ce1792696889c06b354
cd ~/moveit2_ros2_ws/src
vcs import < moveit2/moveit2.repos
# I clone and checkout specific commits instead of using the .repos file
# vcs import < moveit2/moveit2_galactic.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/moveit2_ros2_ws
colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
. ~/px4_ros2_ws/install/local_setup.sh
. ~/moveit2_ros2_ws/install/local_setup.sh
. ~/.bashrc
cd ~/ros2_ws/src
vcs import < moveit_visual_tools/moveit_visual_tools.repos
rosdep install -r --from-paths . --ignore-src --rosdistro galactic -y
cd ~/ros2_ws
colcon build --symlink-install
. ~/px4_ros2_ws/install/local_setup.sh
. ~/moveit2_ros2_ws/install/local_setup.sh
. ~/ros2_ws/install/local_setup.sh
. ~/.bashrc
git clone https://github.com/gstavrinos/uav_inspection_ros2 ~/ros2_ws/src/uav_inspections_ros2
colcon build --symlink-install
