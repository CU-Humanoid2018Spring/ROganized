#!/usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive, FetchGripper, FetchArm, plan_arm, prepare_arm
import argparse
from math import sin, cos, pi
from copy import deepcopy
from roganized_gazebo.table_manager import TableManager
from roganized_grasping.transform import relative_pose
from roganized_rl.utils import GazeboClient
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import subprocess

if __name__ == '__main__':
    rospy.init_node('roganized_demo')
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    parser.add_argument('-f', '--file', type=str, required=True)
    args = parser.parse_args()

    record = []
    with open(args.file) as f:
        for line in f:
            record.append(line[:-1].split(' '))
    print record
    reset_state = ModelState()
    reset_state.model_name = 'fetch'
    reset_state.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
    gazebo = GazeboClient()
    rospy.loginfo('Resetting fetch location...')
    rospy.sleep(1)
    gazebo.set_pose(reset_state)
    prepare_arm()

    table = TableManager()
    rospy.loginfo('Resetting table...')
    rospy.sleep(1)
    table.clear()
    table.spawn()

    for i, loc in enumerate(record[0]):
        print i, loc
        table.move_cube(i, int(loc)/5, int(loc)%5)

    rospy.loginfo('Heuristic score: {}'.format(table.score()))
    for i, loc in record[1:]:
        plan_arm(int(i), int(loc)/5, int(loc)%5)
