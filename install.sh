#!/bin/bash

# ROS Kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base

sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y gazebo7
sudo apt-get install -y libgazebo7-dev
apt install -y ros-kinetic-gazebo-*
gazebo -v

# Catkin tools
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y python-catkin-tools

# Fetch simulator toolchain and download ROganized packages
mkdir -p ~/roganized_ws/src
cd ~/roganized_ws/src
sudo apt-get install -y ros-kinetic-opencv-candidate ros-kinetic-simple-grasping ros-kinetic-moveit* ros-kinetic-navigation ros-kinetic-slam-karto ros-kinetic-costmap-2d ros-kinetic-rgbd-launch
sudo apt-get install -y ros-kinetic-teleop-twist-keyboard
git clone https://github.com/fetchrobotics/robot_controllers.git
git clone https://github.com/fetchrobotics/fetch_gazebo.git -b gazebo7
git clone https://github.com/fetchrobotics/fetch_ros.git
touch fetch_ros/fetch_calibration/CATKIN_IGNORE
git clone https://github.com/mikeferguson/moveit_python.git
git clone https://github.com/mikeferguson/grasping_msgs.git
git clone https://github.com/mikeferguson/simple_grasping.git
git clone https://github.com/ros-perception/open_karto.git
git clone https://github.com/ros-perception/slam_karto.git
git clone https://github.com/ros-perception/sparse_bundle_adjustment.git
git clone https://github.com/ros-planning/moveit.git
git clone https://github.com/ros-planning/geometric_shapes.git
git clone git@github.com:CU-Humanoid2018Spring/ROganized.git
sudo chown -R $USER ~/roganized_ws
cd  ~/roganized_ws
rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
catkin build
echo "source ~/roganized_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
