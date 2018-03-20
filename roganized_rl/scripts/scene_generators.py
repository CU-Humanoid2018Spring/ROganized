#!/usr/bin/env python

# Helper functions for spawning neat and messy scenes.

from __future__ import print_function

import roslib
import actionlib
import sys
import rospy
import numpy as np
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist

roslib.load_manifest('roganized_gazebo')



models = ModelStates()
static_objs = ['table', 'fetch', 'ground_plane', 'camera']
all_objects = sorted(['demo_cube',
                      'cricket_ball',
                      'hammer',
                      'beer_bottle',
                      'parrot_bebop_2'])
MIN_OBJ = 4
MAX_OBJ = 9
center_x = 0
center_y = 2
height = 0.75
dx = 0.35
dy = 0.70


def random_objects(n, selection=all_objects):
    """Pick n random object models by name from the available desk objects. """
    objs_i = np.random.choice(range(len(selection)), size=n, replace=True)
    objs_i.sort()
    return [all_objects[i] for i in objs_i]


def gen_rand_pose(name, x, y, z, dx, dy):
    random_pose = ModelState()
    random_pose.model_name = name
    random_pose.pose.orientation = Quaternion(1, .01, 75, 0)
    random_pose.pose.position.z = z
    random_pose.pose.position.x = x + np.random.uniform(-dx, dx)
    random_pose.pose.position.y = y + np.random.uniform(-dy, dy)
    return random_pose


def random_poses(msg, mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Return dictionary of {name: pos} for publishing. """
    # TODO: Previous models are unused atm, possible to use to generate organized version?
    # TODO: Get static table orientations once.
    # TODO: get table height, table_z is -0.0002 inside msg.pos
    poses = {}
    objs = random_objects(np.random.randint(low=mincount, high=maxcount))
    prev = all_objects[0]
    i = 1
    for name in objs:
        # Track which object clone to reference.
        if name == prev:
            i += 1
        else:
            prev = name
            i = 0
        name += "_clone_" + str(i)
        
        # Generate a pos on the table and save to dict.
        poses[name] = gen_rand_pose(name, center_x, center_y, height, dx, dy)
    return poses


# def main(args):
#
#     rospy.init_node('model_spawner', anonymous=True)
#     # model_callback initiates scene generation
#     rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=1)
#
#     # Setup camera for saving images
#     ic = ImageConverter(img_dir='messy_imgs')
#
#     r = rospy.Rate(20) # 1Hz
#     while not rospy.is_shutdown():
#         # random_pose.pose.position.x = 1 + np.random.uniform(-0.25,0.25)
#         # random_pose.pose.position.y = 0.01 + np.random.uniform(-0.75,0.75)
#         # cup_mover.publish(random_pose)
#         r.sleep()
#
#     # cv2.destroyAllWindows()
#
#
# if __name__ == '__main__':
#     main(sys.argv)
