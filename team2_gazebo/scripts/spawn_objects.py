#!/usr/bin/env python
from __future__ import print_function

import roslib
import actionlib
roslib.load_manifest('team2_gazebo')
import sys
import rospy
import numpy as np
import cv2
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist

models = ModelStates()
# This example shows how to extract model info. from gazebo topics
def model_callback(msg):
  models = msg
  for i, model in enumerate(msg.name):
    if model == 'plastic_cup':
      cup = i
      #print ('Pose of plastic_cup:')
      #print (msg.pose[i])

def main(args):

  rospy.init_node('model_spawner', anonymous=True)
  rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback)

  # Here shows how to manipulate objects using ros publisher
  cup_mover = rospy.Publisher('/gazebo/set_model_state',ModelState, queue_size = 10)
  random_pose = ModelState()
  random_pose.model_name = 'plastic_cup'
  random_pose.pose.orientation = Quaternion(0,0,0,0)
  random_pose.pose.position.z = 0.9 # height of table

  r = rospy.Rate(1) # 1Hz
  while not rospy.is_shutdown():
    random_pose.pose.position.x = np.random.uniform(-0.25,0.25)
    random_pose.pose.position.y = np.random.uniform(-0.75,0.75)
    cup_mover.publish(random_pose)
    r.sleep()



if __name__ == '__main__':
  main(sys.argv)
