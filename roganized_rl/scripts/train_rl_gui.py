#!/usr/bin/env python

# Training the reinforcement learning model
# Author: Yan-Song Chen, Columbia University 2018
import rospy
from utils import MoveBaseClient, FollowTrajectoryClient, GazeboClient,\
                  PointHeadClient, GripperClient, GraspingClient, RL, \
		  ImageSubscriber
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate
from numpy import pi

if __name__ == "__main__":
  # Create a node
  rospy.init_node("training")

  # Make sure sim time is working
  while not rospy.Time.now():
      pass

  # Setup clients
  torso_action = FollowTrajectoryClient("torso_controller",\
                                        ["torso_lift_joint"])
  gripper_action = GripperClient()
  head_action = PointHeadClient()
  grasping_client = GraspingClient()
  gazebo_client = GazeboClient()
  rl_model = RL()
  image_sub = ImageSubscriber()

  rospy.loginfo("Lifting torso...")
  torso_action.move_to([0.4,])

  rospy.loginfo("Looking at table")
  head_action.look_at(0.8, 0.0, 0.0, "base_link")

  ## Gripper pose downward, for future grasp planning
  # downward = Quaternion(0.0,1.57,0.0,1.57)

  rospy.loginfo("Begin learning...")
  count = 100
  import time
  start_time = time.time()
  while not rospy.is_shutdown():
    image = image_sub.get_rgb()
    rl_action = rl_model.action(image)
    new_state = ModelState()
    new_state.model_name =rl_action['name']
    new_state.pose = Pose( Point(rl_action['x'],rl_action['y'], 0.8),\
                           Quaternion(0,0,0,0))
    new_state.pose = rotate(new_state.pose, 0, 0, rl_action['theta'])
    gazebo_client.set_pose(new_state)

    # Wait for the physics to stabilize
    while not gazebo_client.is_stable():
      pass
    #rospy.sleep(0.1)
    count -= 1
    if count == 0:
      break;
  end_time = time.time()
  print ("100 iteration in %s seconds" %(end_time - start_time))
  grasping_client.cancel()
