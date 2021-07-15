#!/bin/bash

starting_dir=$pwd
model=iris
src_path=~/sources/PX4-Autopilot
build_path=$src_path/build/px4_sitl_rtps
world=$src_path/Tools/sitl_gazebo/worlds/empty.world
urdf=~/ros2_ws/src/uav_inspections_ros2/urdf/custom_iris.xacro
# Bypass the urdf with the original sdf for debugging
# urdf=~/sources/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf

rootfs=$build_path/tmp/rootfs # this is the working directory
mkdir -p $rootfs

cd $rootfs

cp $src_path/Tools/posix_lldbinit $rootfs/.lldbinit
cp $src_path/Tools/posix.gdbinit $rootfs/.gdbinit
shift 7
for file in "$@"; do
    cp "$file" $rootfs/
done

pkill -x gazebo || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x px4 || true
pkill -x px4_$model || true

export PX4_SIM_MODEL=$model
. ~/.bashrc
# . /usr/share/gazebo-11/setup.sh
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/sources/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/sources/PX4-Autopilot/Tools/sitl_gazebo/models:~/ros2_ws/src/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/sources/PX4-Autopilot/build/px4_sitl_rtps/build_gazebo
# echo $GAZEBO_MODEL_PATH
# echo $GAZEBO_PLUGIN_PATH
# echo $LD_LIBRARY_PATH
gzserver --verbose $world &

while gz model --verbose --spawn-file=$urdf --model-name=$model -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
    echo "gzserver not ready yet, trying again!"
    sleep 1
done

sleep 5
nice -n 20 gzclient --verbose &

pushd $rootfs >/dev/null

~/sources/PX4-Autopilot/build/px4_sitl_rtps/bin/px4 ~/sources/PX4-Autopilot/build/px4_sitl_rtps/etc -s etc/init.d-posix/rcS -t ~/sources/PX4-Autopilot/test_data

popd >/dev/null

pkill -x gazebo || true
pkill -x gzserver || true
pkill -x gzclient || true
pkill -x px4 || true
pkill -x px4_$model || true
cd $starting_dir
