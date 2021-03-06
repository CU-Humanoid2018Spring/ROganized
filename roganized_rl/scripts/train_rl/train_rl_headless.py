#!/usr/bin/env python

# Training the reinforcement learning model
# Adapted to headless version to that it runs on Google Cloud.

# Author: Yan-Song Chen, Columbia University 2018
import rospy
from roganized_rl.utils import GazeboClient, RL, ImageConverter
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
import time
import numpy as np
from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate
import cv2

def rotate_z(theta):
    rot = pq.Quaternion(axis=[0,0,1],angle=theta)
    q = Quaternion()
    q.x = rot[1]
    q.y = rot[2]
    q.z = rot[3]
    q.w = rot[0]
    return q

if __name__ == "__main__":
  # Create a node
  rospy.init_node("train_rl_headless")

  # Make sure sim time is working
  rospy.loginfo("Syncing rospy time...")
  while not rospy.Time.now():
      pass
  rospy.loginfo("Sync done")

  # Setup clients
  rospy.loginfo("Setting up clients...")
  gazebo_client = GazeboClient()
  rl_model = RL()
  image_sub = ImageConverter()
  rospy.loginfo("Clients ready")

  rospy.loginfo("Begin learning...")
  start_time = time.time()
  count = 100
  while not rospy.is_shutdown():
    image = image_sub.get_rgb()
    while image is None:
        print ('Get image None, retrying...')
        image = image_sub.get_rgb()
        rospy.sleep(0.01)
    cv2.imshow('test',image)
    cv2.waitKey(1)
    rl_action = rl_model.action(image)
    new_state = ModelState()
    new_state.model_name =rl_action['name']
    new_state.pose = Pose( Point(rl_action['x'],rl_action['y'], 0.7),\
                           Quaternion(0,0,0,1))
    #new_state.pose.orientation = rotate_z(rl_action['theta'])
    new_state.pose = rotate(new_state.pose, 0, 0, rl_action['theta'])
    gazebo_client.set_pose(new_state)

    # Wait for the physics to stabilize
    rospy.sleep(0.08)

    count -= 1
    if count == 0:
      break
  cv2.destroyAllWindows()
  rospy.loginfo("Finished training")
  end_time = time.time()
  print ("100 iteration in %s seconds" %(end_time - start_time))
