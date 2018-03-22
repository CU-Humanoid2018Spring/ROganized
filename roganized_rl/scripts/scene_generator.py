#!/usr/bin/env python

# Helper functions for spawning neat and messy scenes.

from __future__ import print_function

import roslib
import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist
from collections import Counter

from utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

roslib.load_manifest('roganized_rl')


models = ModelStates()
fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun']
all_objects = sorted(['wood_cube_5cm',
                      'cricket_ball',
                      'hammer',
                      'beer',
                      'parrot_bebop_2'])
MIN_OBJ = 4
MAX_OBJ = 6
center_x = 2
center_y = 0
height = 0.72
dx = 0.35
dy = 0.35
blank_table = "blank-table.png"


def random_objects(n, selection=all_objects):
    """Return n random sorted model names from the available desk objects. """
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
        
        # Generate a pose on the table and save to dict.
        poses[name] = gen_rand_pose(name, center_x, center_y, height, dx, dy)
    return poses


def neat_linear_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Place objects vertically or horizontally at random, in lines."""
    vert = np.random.random() > 0.5  # Vertical or horizontal
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount)))
    spacing1 = 2*dx / (2 + len(objs))
    for i, (name, count) in enumerate(objs.items()):
        spacing2 = 2*dy / (2 + count)
        for n in range(count):
            name += "_clone_" + str(n)
            a = spacing1 * (i+1)
            b = spacing2 * (n+1)
            # Generate a pose on the table and save to dict.
            poses[name] = gen_pose(name, a if vert else b, b if vert else a, height)
    return poses


def neat_cluster_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """ Cluster into 1-5 evenly spaced groupings on table."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount)))
    clusters = len(objs)
    a = clusters // 2
    b = clusters // 2 + clusters % 2
    centroids = np.mgrid[center_x-dx:center_x+dx:a, center_y-dy:center_y+dy:b].reshape(2, -1).T
    c_dx = dx / a
    c_dy = dy / b
    points = []

    for centroid, (name, count) in zip(centroids, objs.items()):
        c_x, c_y = centroid[0], centroid[1]
        c_i = count // 2
        c_j = count // 2 + count % 2
        c_points = np.mgrid[c_x-c_dx:c_x+c_dx:c_i, c_y-c_dy:c_y+c_dy:c_j].reshape(2, -1).T
        points.append((c_points))
    for name, count, point_list in zip(objs.items(), points):
        for n, (x, y) in zip(range(count), point_list):
            name += "_clone_" + str(n)
            poses[name] = gen_pose(name, x, y, height)
    return poses


def neat_equidist_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a 2D Linspace organization of objects."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount)))
    n = sum([count for count in objs.values()])
    a = n // 2
    b = n // 2 + n % 2
    x_low = center_x-dx
    x_high = center_x+dx
    x_step = 2*dx / a
    y_step = 2*dy 
    print("n, i, j: ", n, a, b)
    points = np.mgrid[center_x-dx:center_x+dx:aj, center_y-dy:center_y+dy:bj].reshape(2, -1).T
    print("points: ", points)
    print("objs.items: ", objs.items())

    for name, count, (x, y) in zip(objs.items(), points):
        name += "_clone_" + str(n)
        poses[name] = gen_pose(name, x, y, height)
    return poses


SCENE_ORGS = {
    'messy': random_poses,
    'neat_linear': neat_linear_poses,
    'neat_cluster': neat_cluster_poses,
    'neat_equidist': neat_equidist_poses

}


def main(args):
    try:
        scene_org = SCENE_ORGS[sys.argv[1]]
        img_dir = sys.argv[2]
        count = sys.maxint if len(sys.argv) < 3 else int(sys.argv[3])
    except:
        print ("Usage: scene_generator.py <%s> <img_dir> <[opt]count>" % '/'.join(SCENE_ORGS.keys()))
        exit()

    print("==== %s SCENE GENERATOR SAVING TO data/%s ====" % (sys.argv[1], img_dir))
    print("Using %s scene generator." % scene_org.__name__)

    rospy.init_node('scene_maker', anonymous=True)

    # Setup GazeboClient
    gc = GazeboClient(obj_mover=scene_org, min_objs=MIN_OBJ, max_objs=MAX_OBJ,
                      fixed_models=fixed_models)

    # Setup camera for saving images
    ic = ImageSubscriber(img_dir=img_dir, count=count, ref_img=blank_table)

    sec_per_img = 0.1
    r = rospy.Rate(1/sec_per_img) # Hz
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
