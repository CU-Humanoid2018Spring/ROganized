#!/usr/bin/env python
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from graspit_commander import GraspitCommander
import rospy
from roganized_rl.utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

def demo():
    gc = GraspitCommander()
    gc.clearWorld()

    # Import models
    mug_pose = Pose(position=Point(0.0,0,0),orientation=Quaternion(0.707,0,0,-0.707))
    table_pose = Pose(position=Point(-0.2,-0.2,0.48),orientation=Quaternion(0,0,0,1))
    gripper_pose = Pose(position=Point(0,0,0.3),orientation=Quaternion(0,0.707,0,0.707))
    # NOTE: in this project, we will only import 1 GraspableBody. Non-target objects
    #       should be imported as an obstacle
    gc.importObstacle('table', pose=table_pose)
    gc.importGraspableBody('mug', pose=mug_pose) 
    gc.importRobot('fetch_gripper',pose=gripper_pose)

    print 'graspable bodies', gc.getGraspableBodies()
    print 'robots', gc.getRobots()
    while not rospy.is_shutdown():
        gc.autoOpen()
        rospy.sleep(0.5)
        gc.autoGrasp()
        rospy.sleep(0.5)

if __name__ == '__main__':
    demo()
