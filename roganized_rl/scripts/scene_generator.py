#!/usr/bin/env python

# Helper functions for spawning neat and messy scenes.

from __future__ import print_function

import roslib
import sys
import rospy
import math
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
    objs = random_objects(np.random.randint(low=mincount, high=maxcount+1))
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
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount+1)))  # objs = {"name": count}
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
            line_pos = line_spacing * (line_i + 1)
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
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount+1)))
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


# def equidist_2d(n, dx, dy, center_x, center_y):
#     step1 = 2*dx / (n//2 + 2)
#     step2 = 2*dy / (n//2 + n%2 + 2)
#     multiples = np.mgrid[1:3:3j, 1:3:3j].reshape(2, -1).T
#     displacements = [(step1*m1, step2*m2) for (m1, m2) in multiples]
#     return [(center_x + x, center_y + y) for (x, y) in displacements]


def neat_equidist_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a 2D linearly spaced organization of objects."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount+1)))
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    n = sum([count for count in objs.values()])
    print("==== number of objs: ", n)

    n = 6

    if n == 4:
        step1 = 2*dx / 4
        step2 = 2*dy / 4
        multiples = [(-1, -1), (1, -1),
                     (-1, 1), (1, 1)]  # central square
    elif n == 5:
        step1 = 2*dx / 4
        step2 = 2*dy / 5
        multiples = [(-1, -1), (1, -1),
                     (0, 0),
                     (-1, 2), (1, 2)]  # box with center
    else:
        step1 = 2*dx / 5
        step2 = 2*dy / 5
        multiples = [(-1, -1), (-1, 0), (-1, 2),
                     (0, -1), (0, 0), (0, 2),
                     (2, -1), (2, 0), (2, 2)]  # 2x3

    points = [(center_x + mx*step1, center_y + my*step2) for (mx, my) in multiples]

    # a = n//2 + 2
    # b = n//2 + n%2 + 2
    # print("a: ", a, " b: ", b)
    # x_step = 2*dx / a
    # y_step = 2*dy / b
    # print("xstep: ", x_step, "ystep: ", y_step)
    # xs = np.arange(center_x - dx, center_x + dx, x_step)[1:-1]
    # ys = np.arange(center_y - dy, center_y + dy, y_step)[1:-1]
    # print('xs', xs)
    # print('ys', ys)
    # X, Y = np.meshgrid(xs, ys)
    # points = np.array([X.flatten(), Y.flatten()]).T
    print("points:")
    for p in points:
        print(p)

    for (name, _), p in zip(objs.items(), points):
        name += "_clone_" + str(n)
        poses[name] = gen_pose(name, p[0], p[1], height)
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
