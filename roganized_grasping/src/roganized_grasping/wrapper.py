#!/usr/bin/env python
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal 
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Quaternion, Pose, Twist, Point, PoseStamped
from graspit_commander import GraspitCommander
import rospy
from roganized_rl.utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose
from roganized_gazebo.table_manager import TableManager
from roganized_grasping.transform import relative_pose
from math import cos, sin, pi

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

def prepare_arm():
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    planning_scene = PlanningSceneInterface("base_link")
    table = TableManager()
    for name, pose in table.models.iteritems():
        if 'cube' in name:
            p = relative_pose(robot_pose, pose)
            planning_scene.addCube(name, 0.045, p.position.x,\
                                   p.position.y, p.position.z)
    planning_scene.addBox("table", 0.5, 0.5, 0.02, 0.55, 0, 0.32)

    gripper_frame = 'gripper_link'
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'
    gripper_pose_stamped.header.stamp = rospy.Time.now()
    gripper_pose_stamped.pose = Pose(Point(0.6, 0, 0.8),\
                                Quaternion(0, 0.707, 0, 0.707))
    move_group.moveToPose(gripper_pose_stamped, gripper_frame)

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
