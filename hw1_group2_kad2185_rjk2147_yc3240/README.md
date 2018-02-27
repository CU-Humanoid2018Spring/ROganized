# Homework 1

## Setup workspace and compile

1. `mkdir -p ~/team2_ws/src`
2. `cd ~/team2_ws/src`
3. `git clone git@github.com:roboticistYan/Humanoid-Team2.git`
4. `cd ~/team2_ws`
5. `catkin build`
6. `echo "source ~/team2_ws/devel/setup.bash" >> ~/.bashrc`
7. `source ~/.bashrc`


## Run Talker and Listener

1. `roslaunch hw1_package hw1.launch`
2. After execution, the output file will be stored in `~/.ros/hw1_output.txt`
