#!/usr/bin/env python
from geometry_msgs.msg import Quaternion, Pose, Point
from roganized_grasping.wrapper import plan_grasp, GraspitPrimitive

def demo():
    mug_pose = None
    mug_pose = Pose(position=Point(0,0,0.5),orientation=Quaternion(0.707,0,0,-0.707))
    table_pose = Pose(position=Point(-0.2,-0.2,0.48),orientation=Quaternion(0,0,0,1))
    gripper_pose = Pose(position=Point(0,0,0.3),orientation=Quaternion(0,0.707,0,0.707))
    pose = plan_grasp(GraspitPrimitive('fetch_gripper', gripper_pose),\
                      GraspitPrimitive('hammer', mug_pose),\
                      [GraspitPrimitive('table', table_pose)])
    return pose
if __name__ == '__main__':
    print 'Result pose: ', demo()
