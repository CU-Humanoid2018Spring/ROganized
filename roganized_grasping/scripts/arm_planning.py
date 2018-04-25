#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive
from roganized_rl.utils import GripperClient
from roganized_rl.utils import GazeboClient
import argparse
from math import sin, cos, pi
from copy import deepcopy
from roganized_gazebo.table_manager import TableManager
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    #parser.add_argument('-c', type=int, required=True)
    parser.add_argument('-x', type=float, required=True)
    parser.add_argument('-y', type=float, required=True)
    #parser.add_argument('-t', '--theta', type=float, required=True)

    args = parser.parse_args()
    #print 'Move cube {} to x={}, y={}'.format(args.c, args.x,args.y)

    #rospy.init_node('arm_demo')

    rospy.loginfo('Waiting for gazebo...')
    table = TableManager()
    gazebo_client = GazeboClient()
    gripper = GripperClient()
    rospy.sleep(1)
    box_pose = gazebo_client.get_pose('simple_cube')
    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.addBox("table", 0.6, 0.6, 0.02, 0.9, 0, 0.34)
    planning_scene.addCube("cube", 0.06, box_pose.position.x, box_pose.position.y, box_pose.position.z)
    #box_pose = table.models['cube_'+str(args.c)]
    #rospy.loginfo('box pose'+str(box_pose))

    # TODO: remove this part once graspit is integrated
    goals = []
    for height in [0.15, 0.03, 0.15]:
        target_pose = deepcopy(box_pose)
        #target_pose.position.x += 0.040
        target_pose.position.z += height
        target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
        goals.append(target_pose)
    #target_pose.orientation = Quaternion(0,0,0,1)

    #box_pose = table._grid_poses[args.x][args.y]
    box_pose = Pose(Point(args.x, args.y, 0.4), Quaternion())

    for height in [0.15, 0.03, 0.15]:
        target_pose = deepcopy(box_pose)
        target_pose.position.z += height
        target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
        goals.append(target_pose)

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

    #gripper_frame = 'wrist_roll_link' #NOTE: use wrist instead of gripper
    gripper_frame = 'gripper_link' #NOTE: use wrist instead of gripper

    count = 0
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
                if count % 3 == 1:
                    gripper.toggle()
                count += 1
            else:
                rospy.logerr('Arm goal in state: %s',\
                               move_group.get_move_action().get_state())
        else:
            rospy.logerr('MoveIt failure. No result returned')
