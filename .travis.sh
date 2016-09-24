#!/bin/bash

set -e

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

apt-get update -qq && apt-get install -qq -y -q wget sudo lsb-release # for docker 

travis_time_start setup.before_install
#before_install:
# Define some config vars.
# Install ROS
sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
# Install ROS
sudo apt-get install -qq -y python-catkin-pkg python-catkin-tools python-rosdep python-wstool ros-$ROS_DISTRO-catkin
source /opt/ros/$ROS_DISTRO/setup.bash
# Setup for rosdep
sudo rosdep init
rosdep update
travis_time_end

travis_time_start setup.install
# Create a catkin workspace with the package under test.
#install:
mkdir -p ~/catkin_ws/src

# Add the package under test to the workspace.
cd ~/catkin_ws/src
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace
travis_time_end

travis_time_start setup.before_script
# Install all dependencies, using wstool and rosdep.
# wstool looks for a ROSINSTALL_FILE defined in before_install.
#before_script:
# source dependencies: install using wstool.
cd ~/catkin_ws/src
wstool init
wstool up

# package depdencies: install using rosdep.
cd ~/catkin_ws
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
travis_time_end

travis_time_start setup.script
# Compile and test.
#script:
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws
catkin build -p1 -j1 --no-status
catkin run_tests -p1 -j1
catkin_test_results --all build
catkin clean -b --yes
catkin config --install
catkin build -p1 -j1 --no-status
travis_time_end
