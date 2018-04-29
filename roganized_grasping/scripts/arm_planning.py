#!/usr/bin/env python

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive, FetchGripper
import argparse
from math import sin, cos, pi
from copy import deepcopy
from roganized_gazebo.table_manager import TableManager
from roganized_grasping.transform import relative_pose

class FetchArm(object):
    def __init__(self):
        self._move_group = MoveGroupInterface('arm_with_torso', 'base_link')
        self._gripper_frame = 'gripper_link'

    def plan_motion(self, target_pose):
        # Convert pose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = target_pose

        # Motion planning request
        self._move_group.moveToPose(gripper_pose_stamped, self._gripper_frame)
        result = self._move_group.get_move_action().get_result()

        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Move Success!')
                return True
            else:
                rospy.logerr('Arm goal in state: %s',\
                               self._move_group.get_move_action().get_state())
                return False
        else:
            rospy.logerr('MoveIt failure. No result returned')
            return False

def main():
    parser = argparse.ArgumentParser(description='Arm Control Pipeline')
    parser.add_argument('-c', type=int, required=True)
    parser.add_argument('-x', type=int, required=True)
    parser.add_argument('-y', type=int, required=True)
    args = parser.parse_args()

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
    #return
    planning_scene.addBox("table", 0.5, 0.5, 0.02, 0.55, 0, 0.32)

    robot_pose = table.models['fetch']
    box_pose = table.models['cube_'+str(args.c)]

    arm = FetchArm()
    robot_pose = table.models['fetch']
    box_pose = table.models['cube_'+str(args.c)]
    target_pose = relative_pose(robot_pose, box_pose)
    target_pose.position.z += 0.15
    target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
    arm.plan_motion(target_pose)

    target_pose.position.z -= 0.12
    arm.plan_motion(target_pose)
    print 'remove cube_{}'.format(args.c)
    planning_scene.removeCollisionObject('cube_{}'.format(args.c))
    gripper.close()

    target_pose.position.z += 0.12
    arm.plan_motion(target_pose)

    #target_pose = deepcopy(table._grid_poses[args.x][args.y])
    robot_pose = table.models['fetch']
    #box_pose = table.models['cube_'+str(args.c)
    box_pose = table._grid_poses[args.x][args.y]
    target_pose = relative_pose(robot_pose, box_pose)
    target_pose.position.z += 0.15
    target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
    arm.plan_motion(target_pose)

    target_pose.position.z -= 0.12
    arm.plan_motion(target_pose)
    gripper.open()

    target_pose.position.z += 0.12
    arm.plan_motion(target_pose)

    name = 'cube_{}'.format(args.c)
    pose = table.models[name]
    planning_scene.addCube(name, 0.045, pose.position.x,\
                           pose.position.y, pose.position.z)
    '''
    target_pose = deepcopy(table._grid_poses[args.x][args.y])
    target_pose.position.z += 0.15
    target_pose.orientation = Quaternion(0,cos(pi/4.),0,sin(pi/4.))
    arm.plan_motion(target_pose)

    target_pose.position.z -= 0.12
    arm.plan_motion(target_pose)
    gripper.open()

    target_pose.position.z += 0.12
    arm.plan_motion(target_pose)
    '''

if __name__ == '__main__':
    main()
    '''
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
    '''
