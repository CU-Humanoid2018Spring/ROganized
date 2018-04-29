#!/usr/bin/env python
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal 
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from graspit_commander import GraspitCommander
import rospy
from roganized_rl.utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

class GraspitPrimitive(object):
    def __init__(self, modelname, pose):
        self.modelname = modelname
        self.pose = pose

def plan_grasp(gripper, target, obstacles):
    '''

    '''
    gc = GraspitCommander()
    gc.clearWorld()

    # Import models
    gc.importRobot(gripper.modelname, pose=gripper.pose)
    gc.importGraspableBody(target.modelname, target.pose)
    for obstacle in obstacles:
        gc.importObstacle(obstacle.modelname, obstacle.pose)
    
    gc.planGrasps()
    return gc.getGraspableBody(0).graspable_body.pose

def demo2():
    mug_pose = Pose(position=Point(0.0,0,0),orientation=Quaternion(0.707,0,0,-0.707))
    table_pose = Pose(position=Point(-0.2,-0.2,0.48),orientation=Quaternion(0,0,0,1))
    gripper_pose = Pose(position=Point(0,0,0.3),orientation=Quaternion(0,0.707,0,0.707))
    pose = plan_grasp(GraspitPrimitive('fetch_gripper', gripper_pose),\
                      GraspitPrimitive('mug', mug_pose),\
                      [GraspitPrimitive('table', table_pose)])
    return pose

class FetchGripper(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", \
                                                   GripperCommandAction)
        rospy.loginfo("Waiting for gripper controller")
        self.client.wait_for_server()
        self.cmd = GripperCommandGoal()
        self.cmd.command.position = 0.06
        self.cmd.command.max_effort = 3.0
        self.client.send_goal(self.cmd)
        self.client.wait_for_result()

    def open(self):
        self.cmd.command.position = 0.06
        self.client.send_goal(self.cmd)
        self.client.wait_for_result()

    def close(self):
        self.cmd.command.position = 0.0
        self.client.send_goal(self.cmd)
        self.client.wait_for_result()
