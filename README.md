# Humanoid Robot Team 2

## Setup workspace and compile

1. `mkdir -p ~/team2_ws/src`
2. `cd ~/team2_ws/src`
3. `git clone git@github.com:roboticistYan/Humanoid-Team2.git`
4. `cd ~/team2_ws`
5. `catkin build`
6. `echo "source ~/team2_ws/devel/setup.bash" >> ~/.bashrc`
7. `source ~/.bashrc`


## Run Talker and Listener

1. `roscore`
2. Open a new terminal `rosrun hw1_package talker.py`
2. Open a new terminal `rosrun hw1_package listener.py`
