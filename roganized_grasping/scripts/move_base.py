#!/usr/bin/env python
import argparse
import rospy
from roganized_rl.utils import MoveBaseClient, GazeboClient
from geometry_msgs.msg import PoseStamped, Pose
from math import sin, cos
from gazebo_msgs.msg import ModelStates, ModelState
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move robot base')
    parser.add_argument('-x', type=float, required=True)
    parser.add_argument('-y', type=float, required=True)
    parser.add_argument('-t', '--theta', type=float, required=True)
    args = parser.parse_args()
    print('x = ', args.x)
    print('y = ', args.y)
    print('theta = ', args.theta)

    rospy.init_node('test_move')
    movebase = MoveBaseClient()
    gz = GazeboClient()
    rospy.sleep(3)
    movebase.goto(args.x, args.y, args.theta)
    print(gz.get_pose('fetch'))
    m = ModelState()
    p = Pose()
    p.position.x = args.x
    p.position.y = args.y
    p.position.z = 0
    p.orientation.z = sin(args.theta / 2.0)
    p.orientation.w = cos(args.theta / 2.0)
    m.pose = p
    m.model_name = 'fetch'
    gz.set_pose(m) 
    rospy.sleep(2)
    print(gz.get_pose('fetch'))
