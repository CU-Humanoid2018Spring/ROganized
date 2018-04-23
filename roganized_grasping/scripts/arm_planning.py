#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive
from roganized_rl.utils import GazeboClient
import argparse
from math import sin, cos, pi
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    parser.add_argument('-x', type=float, required=True)
    parser.add_argument('-y', type=float, required=True)
    parser.add_argument('-t', '--theta', type=float, required=True)

    args = parser.parse_args()
    print 'Receive x={}, y={}, theta={}'.format(args.x,args.y,args.theta)

    rospy.init_node('arm planner')

    rospy.loginfo('Waiting for gazebo...')
    gazebo_client = GazeboClient()
    rospy.sleep(2)
    box_pose = gazebo_client.get_pose('demo_cube')
    rospy.loginfo('target pose'+str(target_pose))
    target_pose.position.z += 0.3
    #target_pose.orientation = Quaternion(0,0,0,1)

    table_pose = Pose(position=Point(-0.2,-0.2,0.48),orientation=Quaternion(0,0,0,1))
    target_pose = plan_grasp(GraspitPrimitive('fetch_gripper', None),\
                             GraspitPrimitive('demo_box', box_pose),\
                             [GraspitPrimitive('table', table_pose)])

    # Fetch Move Group
    move_group = MoveGroupInterface('arm_with_torso', 'base_link')

    #planning_scene = PlanningSceneInterface('base_link')
    planning_scene = PlanningSceneInterface('base_link')
    gripper_frame = 'wrist_roll_link' #NOTE: use wrist instead of gripper

    #gripper_pose = Pose(Point(args.x, args.y, 0.75),\
    #gripper_pose = Pose(Point(args.x, args.y, 1.2),\
    #                    #Quaternion(0,0.707,0,0.707))
    #                    Quaternion(0,cos(args.theta/2.),0,sin(args.theta/2.)))
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = gripper_pose
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
