Homework 2
Group 2
Members: Kelsey Alysha D'Souza kad2185 - Robert Kwiatkowski rjk2147 - Yan-Song Chen yc3240

---
## Part 1
1. 
2. Gazebo simulates a physical world while Rviz is just a visualization software which requires either gazebo or real-world data to render. 
Gazebo purposes (1) CAD-like design tool (2) quick testing (3) AI training in a realistic setting (from: http://gazebosim.org/).
Rviz purposes (1) visualize sensor data (2) visualize topics and other state information (3) provide real-time rendering of a physical robot with selective data displayed (from: http://sdk.rethinkrobotics.com/wiki/Rviz). 
Differences between RViz and Gazebo: (1) Gazebo provides complete world data, whereas Rviz is a purely visualization framework (https://answers.ros.org/question/200044/different-between-gazebo-and-rviz/) (2) Gazebo provides mechanisms for manipulating the world and robots, Rviz does not (https://answers.ros.org/question/200044/different-between-gazebo-and-rviz/) (3) Without a real robot, a gazebo or other compatible simulation must be running and providing data for Rviz to then
process and render (HW2 instructions). 
3. Nodes per cmd1: 9, cmd2: 15, cmd3: 16. Topics per cmd1: , cmd2: , cmd3: .

* Demo by `$roslaunch hw2_group2_kad2185_rjk2147_yc3240 part1.launch $`
* RobotModel, Camera, TF, DepthCloud and Map
* Comparison

|| Rviz | Gazebo |
| ---- | ----- | ----- |
| Main Purpose| Visualize data transfered in ROS server | Model the physic world |
| Naitive Framework | Is a package of ROS | Is a independent project that supports ROS integration |


* The list of nodes after running each command is in Appendix
1. 9 nodes
2. 15 nodes
3. 16 nodes

# Part 2
1.  
2. There are two topics from the depth camera: /head_camera/depth_registered/points and /head_camera/depth_downsample/points. The least number of navigation to view the "Fetch Robotics Logo" is 2. Please see <PICTURE>.


## Appendix

1. Result of running `$rosnodel list` at different stages

```$roslaunch fetch_gazebo playground.launch```
/cmd_vel_mux

/gazebo

/head_camera/crop_decimate

/head_camera/depth_downsample/points_downsample

/head_camera/head_camera_nodelet_manager

/prepare_robot

/publish_base_scan_raw

/robot_state_publisher

/rosout

```$ roslaunch fetch_gazebo_demo demo.launch```
/amcl

/basic_grasping_perception

/cmd_vel_mux

/demo

/gazebo

/head_camera/crop_decimate

/head_camera/depth_downsample/points_downsample

/head_camera/head_camera_nodelet_manager

/map_server

/move_base

/move_group

/publish_base_scan_raw

/robot_state_publisher

/rosout

/tilt_head_node

```$ rosrun rviz rviz -d config/part1.rviz```
/amcl

/basic_grasping_perception

/cmd_vel_mux

/demo

/gazebo

/head_camera/crop_decimate

/head_camera/depth_downsample/points_downsample

/head_camera/head_camera_nodelet_manager

/map_server

/move_base

/move_group

/publish_base_scan_raw

/robot_state_publisher

/rosout

/rviz_1518129331948422024

/tilt_head_node
