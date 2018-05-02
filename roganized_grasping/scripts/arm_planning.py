#!/usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive, FetchGripper, FetchArm
import argparse
from math import sin, cos, pi
from copy import deepcopy
from roganized_gazebo.table_manager import TableManager
from roganized_grasping.transform import relative_pose

def plan_arm(c, x, y):

    rospy.loginfo('Initializing...')
    table = TableManager()
    gripper = FetchGripper()
    rospy.sleep(1)

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    robot_pose = table.models['fetch']
    for name, pose in table.models.iteritems():
        if 'cube' in name:
            p = relative_pose(robot_pose, pose)
            planning_scene.addCube(name, 0.045, p.position.x,\
                                   p.position.y, p.position.z)
    planning_scene.addBox("table", 0.5, 0.5, 0.02, 0.55, 0, 0.32)

    robot_pose = table.models['fetch']
    box_pose = table.models['cube_'+str(c)]

    arm = FetchArm()
    robot_pose = table.models['fetch']
    box_pose = table.models['cube_'+str(c)]
    target_pose = relative_pose(robot_pose, box_pose)
    target_pose.position.z += 0.15
    target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
    arm.plan_motion(target_pose)

    target_pose.position.z -= 0.12
    arm.plan_motion(target_pose)
    rospy.loginfo('Remove cube_{}'.format(c))
    planning_scene.removeCollisionObject('cube_{}'.format(c))
    gripper.close()

    target_pose.position.z += 0.12
    arm.plan_motion(target_pose)

    robot_pose = table.models['fetch']
    box_pose = table._grid_poses[x][y]
    target_pose = relative_pose(robot_pose, box_pose)
    target_pose.position.z += 0.15
    target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
    arm.plan_motion(target_pose)

    target_pose.position.z -= 0.12
    arm.plan_motion(target_pose)
    gripper.open()

    target_pose.position.z += 0.12
    arm.plan_motion(target_pose)

    name = 'cube_{}'.format(c)
    pose = table.models[name]
    planning_scene.addCube(name, 0.045, pose.position.x,\
                           pose.position.y, pose.position.z)

if __name__ == '__main__':
    rospy.init_node('arm_planning')
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    parser.add_argument('-c', type=int, required=True)
    parser.add_argument('-x', type=int, required=True)
    parser.add_argument('-y', type=int, required=True)
    args = parser.parse_args()
    plan_arm(args.c, args.x, args.y)
