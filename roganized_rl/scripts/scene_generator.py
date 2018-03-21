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

from utils import ImageSubscriber, GazeboClient, gen_rand_pose

roslib.load_manifest('roganized_rl')


models = ModelStates()
fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun']
all_objects = sorted(['wood_cube_5cm',
                      'cricket_ball',
                      'hammer',
                      'beer_bottle',
                      'parrot_bebop_2'])
MIN_OBJ = 4
MAX_OBJ = 6
center_x = 2
center_y = 0
height = 0.72
dx = 0.35
dy = 0.35


def random_objects(n, selection=all_objects):
    """Pick n random object models by name from the available desk objects. """
    objs_i = np.random.choice(range(len(selection)), size=n, replace=True)
    objs_i.sort()
    return [all_objects[i] for i in objs_i]


def random_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
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


SCENE_ORGS = {
    'messy': random_poses,
    'neat_linear': None,
    'neat_cluster': None,
    'neat_equidist': None

}


def main(args):
    try:
        scene_org = SCENE_ORGS[sys.argv[1]]
        img_dir = sys.argv[2]
    except:
        print ("Usage: scene_generator.py <%s> <img_dir>" % '/'.join(SCENE_ORGS.keys()))
        exit()

    print("==== %s SCENE GENERATOR SAVING TO data/%s ====" % (sys.argv[1], img_dir))
    print("Using %s scene generator." % scene_org.__name__)

    rospy.init_node('scene_maker', anonymous=True)

    # Setup GazeboClient
    gc = GazeboClient(obj_mover=scene_org, min_objs=MIN_OBJ, max_objs=MAX_OBJ,
                      fixed_models=fixed_models)

    # Setup camera for saving images
    ic = ImageSubscriber(img_dir=img_dir)
#
    r = rospy.Rate(1/.2) # 1Hz
    while not rospy.is_shutdown():
#         # random_pose.pose.position.x = 1 + np.random.uniform(-0.25,0.25)
#         # random_pose.pose.position.y = 0.01 + np.random.uniform(-0.75,0.75)
#         # cup_mover.publish(random_pose)
        r.sleep()
#
#     # cv2.destroyAllWindows()
#
#
if __name__ == '__main__':
    main(sys.argv)
