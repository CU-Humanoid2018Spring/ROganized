#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

# Redistributed for ROganize, an open-source project of reinforcement learning
# at Columbia University. The above disclaimer is preserved per thr original
# author.

# Author: Yan-Song Chen

import actionlib
import cv2
import rospy
import os

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from grasping_msgs.msg import FindGraspableObjectsAction, \
    FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" \
                                                   % name, \
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

class GripperClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", \
                                                   GripperCommandAction)
        rospy.loginfo("Waiting for gripper controller")
        self.client.wait_for_server()
        self.open = True
        self.cmd = GripperCommandGoal()
        self.cmd.command.position = 0.3
        self.cmd.command.max_effort = 2.0
        self.client.send_goal(self.cmd)
        self.client.wait_for_result()

    def toggle(self):
        if self.open:
            self.cmd.command.position = 0.0
        else:
            self.cmd.command.position = 0.3
        self.open = not self.open
        self.client.send_goal(self.cmd)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head" \
                                                   ,PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):
    def __init__(self):
        # Create move group interface for a fetch robot
        self.move_group = MoveGroupInterface('arm_with_torso', 'base_link')
        self.gripper_frame = 'wrist_roll_link'
        self.gripper_pose_stamped = PoseStamped()
        self.gripper_pose_stamped.header.frame_id = 'base_link'

        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)

    def move_gripper(self, pose):
        self.gripper_pose_stamped.header.stamp = rospy.Time.now()
        self.gripper_pose_stamped.pose = pose
        self.move_group.moveToPose(self.gripper_pose_stamped, self.gripper_frame)
        result = self.move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    def cancel(self):
        self.move_group.get_move_action().cancel_all_goals()

# Tool for resetting object position within a region.
def gen_rand_pose(name, x, y, z, dx, dy):
    random_pose = ModelState()
    random_pose.model_name = name
    random_pose.pose.orientation = Quaternion(1, .01, 75, 0)
    random_pose.pose.position.z = z
    random_pose.pose.position.x = x + np.random.uniform(-dx, dx)
    random_pose.pose.position.y = y + np.random.uniform(-dy, dy)
    return random_pose


from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
class GazeboClient:
    def __init__(self, obj_mover, min_objs, max_objs,
                 fixed_models={'table', 'fetch', 'ground_plane', 'camera'}):
        '''Initialize with a mover function to update scene with desired neat/messy algorithm.'''
        self.models = None
        self.sub = rospy.Subscriber('/gazebo/model_states',
                                    ModelStates, self.model_callback)
        self.pub = rospy.Publisher('/gazebo/set_model_state',
                                   ModelState, queue_size = 10)
        self.obj_mover = obj_mover  # Function for generating new object position.
        self.fixed_models = fixed_models
        self.mincount = min_objs
        self.maxcount = max_objs

    def model_callback(self, msg):
        # Initialize models
        if self.models is None:
            self.models={}
            for i, name in enumerate(msg.name):
                if name in self.fixed_models:
                    pass
                else:
                    rospy.loginfo("Add model name: %s", name)
                    self.models[name] = msg.pose[i]

        # Update existing models with new poses
        else:
            self.reset(msg.name)
            new_poses = self.obj_mover(mincount=self.mincount, maxcount=self.maxcount)
            # old_poses = {name: msg.pose[i] for i, name in enumerate(msg.name) if name in self.fixed_models}
            # self.models = old_poses.update(new_poses)
            for name, pos in new_poses.items():
                self.pub.publish(pos)
            # for i, name in enumerate(msg.name):
            #     if name in self.models:
            #         self.models[name] = msg.pose[i]
            #     elif name in self.fixed_models:
            #         self.models[name] = self.obj_mover(name)
            #     else:
            #         rospy.logerr("Model name %s does not exist", name)

    def reset(self, names):
        for o in names:
            # Skip objects we want to keep in the scene, e.g. tables.
            if o in self.fixed_models:
                continue
            random_pose = gen_rand_pose(o, -5, -5, 0, 3, 3)
            self.pub.publish(random_pose)
        # default_state = ModelState()
        # if self.models is None:
        #     rospy.logerr("models is None")
        #     return
        #default_state.model_name = 'table1'
        #default_state.pose.orientation = Quaternion(0,0,0,0)
        #default_state.pose.position = Point(0.8, 0, 0.00)
        #self.pub.publish(default_state)


    def get_pose(self, name):
        return self.models[name]

    def set_pose(self, state):
        if state.model_name in self.models:
            self.pub.publish(state)
        else:
            rospy.logerr("Model name %s doesn't exist", state.model_name)

###############################################################################
#TODO: REINFORCEMENT LEARNING CODE START HERE
################################################################################
import numpy as np
actions = []
actions.append({'name':'demo_cube', 'x': 1.2, 'y':  0.25, 'theta': np.pi/6.0})
actions.append({'name':'demo_cube', 'x': 1.2, 'y': -0.25, 'theta': np.pi/4.0})
i = -1
class RL(object):
    def __init__(self):
        pass

    def action(self, image = None):
        action = {}
        action['name'] = np.random.choice(['bowl','demo_cube','cricket_ball'])
        action['x'] = np.random.uniform(0.8,1.2)
        action['y'] = np.random.uniform(-0.25,0.25)
        action['theta'] = np.pi / np.random.uniform(1.0, 6.0)
        return action
################################################################################
# END OF REINFORCEMENT LEARNING CODE
################################################################################

###############################################################################
#TODO: IMAGE PROCESSING CODE START HERE
################################################################################
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class ImageSubscriber(object):
    '''Capture images from specified camera feed, saving to specified subdirectory of data/.'''
    def __init__(self, feed='/camera/rgb/image_raw', img_dir='messy_imgs',
                 prefix='scene', suffix='.png'):

        # CvBridge and subscriber to access images
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(feed, Image, self.save_image)

        # View image feed via publisher
        # self.image_pub = rospy.Publisher("/team2/output_image", Image, queue_size=10)

        # Establish directory location for saving images
        cwd = os.getcwd()
        data_dir = 'data'
        if os.path.basename(cwd) == 'Humanoid-Team2':
            path = os.path.join(os.getcwd(), data_dir)
        elif os.path.basename(cwd) == 'team2_ws':
            path = os.path.join(os.getcwd(), "src/Humanoid-Team2", data_dir)
        else:
            path = data_dir

        self.img_dir = os.path.join(path, img_dir)
        self.prefix = prefix
        self.suffix = suffix

        # Make team2_ws/src/Humanoid-Team2/data/img_dir if does not already exist.
        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)
            print("Making path to ", self.img_dir)


        print("ImageSubscriber initialized to save to: %s" % self.img_dir)


    def get_rgb(self):
        return None


    def get_depth(self):
        pass


    def save_image(self, data):
        print("...ImageSubscriber saving image to ", self.img_dir)
        # Callback for capturing and saving image from self.image_sub feed.
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        n = len(os.listdir(self.img_dir))
        img_path = os.path.join(self.img_dir, self.prefix + str(n) + self.suffix)
        cv2.imwrite(img_path, cv_image)

################################################################################
# END OF IMAGE PROCESSING CODE
################################################################################
