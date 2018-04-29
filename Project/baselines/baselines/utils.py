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
        move_goal.target_pose.pose.orientation.z = sin(theta / 2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta / 2.0)
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
        self.cmd.command.max_effort = 1.2
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
                                                   , PointHeadAction)
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


# Tools for generating poses
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point


# Create a pose given name and coordinates, with default orientation.
def gen_pose(name, x, y, z, orient=Quaternion(1, .01, 75, 0)):
    # TODO which quaternion for horizontal vs vertical hammer?
    random_pose = ModelState()
    random_pose.model_name = name
    random_pose.pose.orientation = orient
    random_pose.pose.position.z = z
    random_pose.pose.position.x = x
    random_pose.pose.position.y = y
    return random_pose


# Create random pose within a bounding x and y region.
def gen_rand_pose(name, x, y, z, dx, dy):
    return gen_pose(name, x + np.random.uniform(-dx, dx),
                    y + np.random.uniform(-dy, dy), z)


class GazeboClient:
    def __init__(self, obj_mover=None, min_objs=None, max_objs=None,
                 fixed_models={'table', 'fetch', 'ground_plane', 'camera'}):
        '''Option to initialize with:
            - scene generating function, 
            - interval of objects to produce per scene, and 
            - [opt] set of fixed models. '''

        self.models = None
        self.sub = rospy.Subscriber('/gazebo/model_states',
                                    ModelStates, self.model_callback)
        self.pub = rospy.Publisher('/gazebo/set_model_state',
                                   ModelState, queue_size=10)
        self.obj_mover = obj_mover  # Function for generating new object position.
        self.fixed_models = fixed_models

        self.simple_client = (obj_mover == None)
        self.mincount = min_objs
        self.maxcount = max_objs
        self.stable = False
        self.names = None # Objects in the camera's view

    def model_callback(self, msg):
        if self.models is None:  # Initialize models if not yet done
            self.models = {}
            for i, name in enumerate(msg.name):
                if name in self.fixed_models:
                    pass
                else:
                    rospy.loginfo("Add model name: %s", name)
                    self.models[name] = msg.pose[i]
        else:
            for i, name in enumerate(msg.name):
                if name in self.models:
                    self.models[name] = msg.pose[i]
                elif name == 'fetch':
                    self.robot_pose = msg.pose[i]
                elif name in self.fixed_models:
                    #rospy.logwarn("Model name %s is fixed", name)
                    pass
                else:
                    rospy.logerr("Model name %s does not exist", name)

    def generate_scene(self):
        # Update existing models with new poses and publish
        new_poses = self.obj_mover(mincount=self.mincount, maxcount=self.maxcount)
        for name, pos in new_poses.items():
            self.pub.publish(pos)
        self.names = new_poses.keys()

    def reset(self):
        default_state = ModelState()
        if self.models is None:
            rospy.logerr("models is None")
            return

    def mover_reset(self):
        """Replace objects on table to corner."""
        if not self.names:
            return
        for name in self.names:
            # Skip objects we want to keep in the scene, e.g. tables.
            if name in self.fixed_models:
                continue
            random_pose = gen_rand_pose(name, -5, -5, 0, 3, 3)
            self.pub.publish(random_pose)

    def full_reset(self):
        for name in self.models.keys():
            # Skip objects we want to keep stationary in the scene, e.g. table.
            if name in self.fixed_models:
                continue
            random_pose = gen_rand_pose(name, -5, -5, 0, 3, 3)
            self.pub.publish(random_pose)

    def get_pose(self, name):
        if name == 'fetch':
            return self.robot_pose
        return self.models[name]

    def set_pose(self, state):
        if self.models and state.model_name in self.models:
            self.pub.publish(state)
        elif state.model_name == 'fetch':
            self.pub.publish(state)
        else:
            rospy.logerr("Model name %s doesn't exist", state.model_name)

    def get_rl_state(self):
        rl_state = []
        for name in self.names:
            p = self.models[name].position
            q = self.models[name].orientation
            rl_state.append([name, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        return rl_state

###############################################################################
# TODO: REINFORCEMENT LEARNING CODE START HERE
################################################################################
import numpy as np

actions = []
actions.append({'name': 'demo_cube', 'x': 1.2, 'y': 0.25, 'theta': np.pi / 6.0})
actions.append({'name': 'demo_cube', 'x': 1.2, 'y': -0.25, 'theta': np.pi / 4.0})
i = -1


class RL(object):
    def __init__(self):
        pass

    def action(self, image=None):
        action = {}
        action['name'] = np.random.choice(['bowl', 'demo_cube', 'cricket_ball'])
        action['x'] = np.random.uniform(0.8, 1.2)
        action['y'] = np.random.uniform(-0.25, 0.25)
        action['theta'] = np.pi / np.random.uniform(1.0, 6.0)
        return action


################################################################################
# END OF REINFORCEMENT LEARNING CODE
################################################################################

###############################################################################
# TODO: IMAGE PROCESSING CODE START HERE
################################################################################
def same_img(img, ref):
    """Check if cv2 images are identical."""
    identical = (img.shape == ref.shape) and not (np.bitwise_xor(img, ref).any())
    # if identical:
    #     print("same image detected")
    return identical


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageSubscriber(object):
    '''Captures images from specified camera feed and saves to specified subdirectory of data/.'''

    def __init__(self, img_dir=None, feed='/camera/rgb/image_raw', ref_imgs=None,
                 batch_size=100, prefix='scene', suffix='.png'):
        """
          - img_dir: subdirectory of /data to save images to
          - count: number of images to produce/save
          - feed: camera feed to subscribe to
          - ref_img: reference image to ignore when producing images, e.g. scene with a blank table
          - batch_size: max number of images to save per subdirectory (default 100)
          - prefix, suffix: image file <prefix>_<number>.<suffix>
        """

        self.msg = None
        self.simple_subscriber = img_dir == None
        if self.simple_subscriber:
            return

        # CvBridge and camera feed subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(feed, Image, self.callback_function)

        # Publisher to view image feed
        # self.image_pub = rospy.Publisher("/team2/output_image", Image, queue_size=10)

        # Setup directory within /data for saving images, using current location. Creates /data if necessary.
        cwd = os.getcwd()
        data_dir = 'data'
        self.img_dir = img_dir
        if os.path.basename(cwd) == 'Humanoid-Team2':
            self.data_path = os.path.join(os.getcwd(), data_dir)
        elif os.path.basename(cwd) == 'team2_ws':
            self.data_path = os.path.join(os.getcwd(), "src/Humanoid-Team2", data_dir)
        else:
            self.data_path = data_dir

        # Make team2_ws/src/Humanoid-Team2/data/<img_dir> if does not exist
        if not os.path.exists(os.path.join(self.data_path, self.img_dir)):
            os.makedirs(os.path.join(self.data_path, self.img_dir))
            print("Making path to ", os.path.join(self.data_path, self.img_dir))

        # Make .../img_dir/batch_0 if does not already exist.
        self.batch_num = 0
        self.update_cur_dir()
        print("==== CUR_DIR: ", self.cur_dir)

        self.img_count = len(os.listdir(self.cur_dir))
        self.batch_size = batch_size
        self.prefix = prefix
        self.suffix = suffix
        self.ref_imgs = [cv2.imread(os.path.join(self.data_path, ref_img)) for ref_img in ref_imgs] if ref_imgs else []

        print("ImageSubscriber %s ignoring image: %s" % (self.ref_imgs != [], os.path.join(self.data_path, ref_img)))
        print("ImageSubscriber initialized to save images to: %s" % self.img_dir)

    def update_cur_dir(self):
        """Create new directory for next batch of images."""
        self.cur_dir = os.path.join(self.data_path, self.img_dir, "batch_" + str(self.batch_num))
        if not os.path.exists(self.cur_dir):
            os.makedirs(self.cur_dir)
            print("Making batch directory: ", self.cur_dir)

    def get_rgb(self):
        return None

    def get_depth(self):
        pass

    def callback_function(self, msg):
        self.msg = msg

    def add_ref(self, img_name):
        # print("ImageSubscriber now ignoring: ",os.path.join(self.cur_dir, img_name))
        self.ref_imgs.append(cv2.imread(os.path.join(self.cur_dir, img_name)))

    def pop_ref(self):
        self.ref_imgs.pop()

    def save_image(self):
        """
        Capture and save image from self.image_sub feed.
        Ignores images which match self.ref_img.
        Returns True if successfully saved.
        """
        self.cur_img = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")
        n = len(os.listdir(self.cur_dir))
        # Create a new batch of images directory
        if self.img_count > 1 and n % self.batch_size == 0:
            self.batch_num += 1
            self.update_cur_dir()
        # Save image if different from the reference images
        if sum([same_img(self.cur_img, ref) for ref in self.ref_imgs]) == 0:
            img_name = self.prefix + str(self.img_count) + self.suffix
            img_path = os.path.join(self.cur_dir, img_name)
            cv2.imwrite(img_path, self.cur_img)
            self.img_count += 1
            print("...ImageSubscriber saved: ", img_path)
            return img_name
        return ""

    def check_blank(self):
        """Check if current scene matches ref_imgs[:-1] (excluding previous scene)."""
        return sum([same_img(self.cur_img, ref) for ref in self.ref_imgs[:-1]]) > 0
        # return True
################################################################################
# END OF IMAGE PROCESSING CODE
################################################################################
class ImageConverter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
    self.cv_image = None
  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def get_rgb(self):
    if self.cv_image is None:
      return None
    return self.cv_image.copy()
