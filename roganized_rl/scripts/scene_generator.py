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
# TODO: all-caps the constants below
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
    # TODO: Get static table orientations once, including height.
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
    """Place objects in vertical or horizontal lines, one type of object per line."""
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount)))  # objs = {"name": count}
    # TODO: shuffle objs randomly
    # Determine spacing along x and y dimensions
    xdim = 2*dx 
    ydim = 2*dy
    line_spacing = (xdim if vert else ydim) / (2 + len(objs))  # Spacing between lines, buffered from table edge
    # Compute pose per object, per line, and save to poses dict
    for line_i, (name, count) in enumerate(objs.items()):
        obj_spacing = (ydim if vert else xdim) / (2 + count)  # Spacing between objects in line, buffered from table edge
        for obj_i in range(count):
            name += "_clone_" + str(obj_i)
            line_pos = spacing1 * (line_i + 1)
            obj_pos = obj_spacing * (obj_i + 1)
            # Generate pose and save to dict.
            poses[name] = gen_pose(name=name, 
                                   x=line_spacing if vert else obj_spacing, 
                                   y=obj_spacing if vert else line_spacing, 
                                   z=height)
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
    for (name, count), point_list in zip(objs.items(), points):
        for n, (x, y) in zip(range(count), point_list):
            name += "_clone_" + str(n)
            poses[name] = gen_pose(name, x, y, height)
    return poses


def neat_equidist_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a 2D Linspace organization of objects."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount)))
    n = sum([count for count in objs.values()])
    a = n // 2 + 2
    b = n // 2 + n % 2 + 2
    x_step = 2*dx / a
    y_step = 2*dy / b
    xs = np.arange(center_x - dx + x_step, center_x + dx - x_step, x_step)
    ys = np.arange(center_y - dy + y_step, center_y + dy - y_step, y_step)
    X, Y = np.meshgrid(xs, ys)
    points = np.array([X.flatten(), Y.flatten()]).T
    print("points: ", points)
    print("objs.items: ", objs.items())

    for (name, count), (x, y) in zip(objs.items(), points):
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
  # Usage example: python scene_generator.py neat_cluster clustered 100
    try:
        scene_org = SCENE_ORGS[sys.argv[1]]
        img_dir = sys.argv[2]
        count = 150 if len(sys.argv) < 4 else int(sys.argv[3])
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

    # Establish image capture rate
    sec_per_img = 0.1
    r = rospy.Rate(1/sec_per_img) # Hz
    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
