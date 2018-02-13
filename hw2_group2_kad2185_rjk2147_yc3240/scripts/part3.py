#!/usr/bin/python
from graspit_commander import GraspitCommander
from geometry_msgs.msg import Pose, Point, Quaternion
import time

gc = GraspitCommander()

gripper = gc.importRobot('fetch_gripper')
gripper_pose = Pose( position=Point(0.06,-0.20,0), orientation=Quaternion(0,0,1.1,1))
gc.setRobotPose(gripper_pose,id=0)

mug = gc.importGraspableBody('mug')

gc.planGrasps()

# Show Grasp Pose
for i in range(5):
	gc.autoOpen()
	time.sleep(1)
	gc.autoGrasp()
	time.sleep(1)
