sudo apt update && sudo apt install curl gnupg lsb-release libgmock-dev -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-galactic-desktop ros-galactic-rqt* ros-galactic-eigen3-cmake-module python3-colcon-common-extensions python3-rosdep python3-empy -y
. /opt/ros/galactic/setup.bash
sudo rosdep init
rosdep update
echo "alias rosdep_install='rosdep install -i --from-path src --rosdistro galactic -y'" >> ~/.bashrc
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=~/ros2_ws" >> ~/.bashrc
. /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
rm -rf uav_inspection_ros2
rm -rf px4_ros_com
rm -rf px4_msgs
git clone git@github.com:gstavrinos/uav_inspection_ros2
git clone https://github.com/PX4/px4_ros_com
git clone https://github.com/PX4/px4_msgs
. ~/.bashrc
mkdir ~/sources
cd ~/sources
sudo apt install gradle -y
rm -rf foonathan_memory_vendor
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build
cd build
cmake ..
sudo cmake --build . --target install
rm -rf ~/sources/FastDDS-2.0.0 
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.0 ~/sources/FastDDS-2.0.0
cd ~/sources/FastDDS-2.0.0
mkdir build
cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
rm -rf ~/sources/Fast-RTPS-Gen 
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/sources/Fast-RTPS-Gen
cd ~/sources/Fast-RTPS-Gen
./gradlew assemble
sudo ./gradlew install
cd ~/ros2_ws/
cd ~/ros2_ws/src/px4_ros_com/scripts
sudo bash build_ros2_workspace.bash
sudo pip3 install -U pyros-genmsg setuptools
