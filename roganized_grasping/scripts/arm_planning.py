#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive
from roganized_rl.utils import GazeboClient
import argparse
from math import sin, cos, pi
from copy import deepcopy
from roganized_gazebo.table_manager import TableManager
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    parser.add_argument('-x', type=float, required=True)
    parser.add_argument('-y', type=float, required=True)
    parser.add_argument('-t', '--theta', type=float, required=True)

    args = parser.parse_args()
    print 'Receive x={}, y={}, theta={}'.format(args.x,args.y,args.theta)

    rospy.init_node('arm_demo')

    rospy.loginfo('Waiting for gazebo...')
    table = TableManager()
    #gazebo_client = GazeboClient()
    #rospy.sleep(1)
    box_pose = deep_copy(table.models['cube_0'])
    rospy.loginfo('box pose'+str(box_pose))

    # TODO: remove this part once graspit is integrated
    goals = []
    for height in [0.15, 0.03]:
        target_pose = deepcopy(box_pose)
        target_pose.position.z += height
        target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
        goals.append(target_pose)
    #target_pose.orientation = Quaternion(0,0,0,1)

    # Graspit Pipeline
    #table_pose = Pose(position=Point(-0.2,-0.2,0.48),orientation=Quaternion(0,0,0,1))
    #table_pose = Pose(position=Point(0.6,-0.8,0.95),orientation=Quaternion(0,0,0,1))
    #target_pose = plan_grasp(GraspitPrimitive('fetch_gripper', None),\
    #                         GraspitPrimitive('wood_cube_5cm', box_pose),\
    #                         [GraspitPrimitive('table', table_pose)])
    rospy.loginfo('target pose'+str(target_pose))

    # Arm Trajectory Planning Pipeline
    # Fetch Move Group
    move_group = MoveGroupInterface('arm_with_torso', 'base_link')

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.addBox("table", 1, 1, 0.02, 0.9, 0, 0.45)
    planning_scene.addCube("cube", 0.07, box_pose.position.x, box_pose.position.y, box_pose.position.z)
    #gripper_frame = 'wrist_roll_link' #NOTE: use wrist instead of gripper
    gripper_frame = 'gripper_link' #NOTE: use wrist instead of gripper

    for target_pose in goals:
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = target_pose
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Move Success!')
            else:
                rospy.logerr('Arm goal in state: %s',\
                               move_group.get_move_action().get_state())
        else:
            rospy.logerr('MoveIt failure. No result returned')
