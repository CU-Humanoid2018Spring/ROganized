#!/usr/bin/env python
###
# Helper functions for spawning neat and messy scenes.
# Run: $ rosrun scene_generator.py four_cube four_cube_imgs 30000
#
###

from __future__ import print_function

import sys
import random
import rospy
import itertools
import math
import numpy as np
from gazebo_msgs.msg import ModelStates
from collections import Counter

from roganized_rl.utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

models = ModelStates()
FIXED_MODELS = ['table', 'fetch', 'ground_plane', 'camera', 'sun']
ALL_OBJECTS = sorted(['wood_cube_5cm',
                      'cricket_ball',
                      'green_hammer',
                      'pink_beer',
                      'demo_cube'])
MIN_OBJ = 4
MAX_OBJ = 6
# TODO: all-caps the constants below
CENTER_X = 2.
CENTER_Y = 0.
# cafe_table dimensions: 2ft x 2ft tabletop,
TABLE_HEIGHT = 0.72
TABLE_WIDTH = .58
HALF_WIDTH = TABLE_WIDTH/2
TABLE_LENGTH = .58
HALF_LENGTH = TABLE_LENGTH/2
BLANK_TABLES = ["blank-table.png", "blank-table-2.png", "blank-table-3.png", "blank-table-4.png"]


def random_objects(n, selection=ALL_OBJECTS):
    """Return n random model names from the specified object list, grouped by model type. """

    # Get indices of n objects from list, with resampling.
    objs_i = np.random.choice(range(len(selection)), size=n, replace=True)

    # Map index list to (model, count) pairs: [i1, i2..] => [(model_x, count), ..]
    objs = Counter([ALL_OBJECTS[i] for i in objs_i]).items()  # Counter{"name": count}
    print("*******selected objects: ", (objs))

    # Return actual names, e.g. [(model_x, 2)] => [(model_x_0, model_x_1)]
    names = []
    for name, count in objs:
        group = []
        for i in range(count):
            group.append(name + "_clone_" + str(i))
        names.append(group)
    # print("*******names: ", names)

    random.shuffle(names)

    return names


def random_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Return dictionary of {name: pos} for publishing. """
    poses = {}
    model_names = random_objects(np.random.randint(low=mincount, high=maxcount + 1))
    for group in model_names:
        for name in group:
            # Generate a pose on the table and save to dict.
            poses[name] = gen_rand_pose(name, CENTER_X, CENTER_Y, TABLE_HEIGHT, TABLE_WIDTH - .1, TABLE_LENGTH - .1)
    return poses

  
CUBES = ['cube_' + str(i) for i in range(4)]
def random_cube_poses(n=4, mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Return dictionary of {cube_name: pos} for publishing. """
    poses = {}
    for model_name in CUBES:
        # Generate a pose on the table and save to dict.
        poses[model_name] = gen_rand_pose(model_name, CENTER_X, CENTER_Y, TABLE_HEIGHT, TABLE_WIDTH - .1, TABLE_LENGTH - .1)
    return poses
  

def linear_points(objs, cx=CENTER_X, cy=CENTER_Y,
                  dx=HALF_WIDTH, dy=HALF_LENGTH, types_along_x=True):
    """
        Returns list of (x, y) points, spaced by:
        (1) number of model types, and
        (2) number of models of that type.
        Orientation based on value of types_along_x.
    :return:
    """
    # Return
    points =[]

    # c1, d1 = center and length of types-spaced dimension (for row spacing)
    # c2, d2 = center and length of object-spaced dimension (for object spacing within row)
    c1, c2, d1, d2 = (cx, cy, dx, dy) if types_along_x else (cy, cx, dy, dx)

    n_types = len(objs)
    print("n_types: ", n_types)
    row_start = c1 - d1*.9
    row_end = c1 + d1*.9
    rows = np.linspace(row_start, row_end, n_types)
    for row, group in zip(rows, objs):
        o_start = c2 - d2*.9
        o_end = c2 + d2*.9
        obj_points = np.linspace(o_start, o_end, len(group))

        points = points + list((row, o) for o in obj_points)

    if not types_along_x:
        points = [(x, y) for y, x in points]

    print("****LINEAR POINTS: ", points)
    return points


def neat_linear_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """
        Place objects in vertical or horizontal lines, one type of object per line.
    """

    # Random selection of objects [((x_clone_0), (x_clone_1)), (y_clone_0), ...]
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    # Random vertical/horizontal row orientation
    orient = np.random.random() > 0.5
    # print("==== neat_linear %s" % ('x rows' if orient else 'y rows'))

    # Generate objects points in obj order with specified orientation
    points = linear_points(objs, types_along_x=orient)

    # Create poses from names and points.
    poses = {}
    for name, p in zip(itertools.chain(*objs), points):
        poses[name] = gen_pose(name, p[0], p[1], TABLE_HEIGHT)


    # print(poses)
    return poses


def polygon_points(sides, radius=HALF_LENGTH*.9, rotation=0, cx=CENTER_X, cy=CENTER_Y):
    """
        Returns list of (x, y) points for n-sided polygon around given center.
        Rotation given in radians.
    """
    one_segment = math.pi * 2 / sides

    points = [
        (math.sin(one_segment * i + rotation) * radius,
         math.cos(one_segment * i + rotation) * radius)
        for i in range(sides)]

    translated_points = [(cx + x, cy + y) for (x, y) in points]

    return translated_points


def neat_polygon_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a circularly spaced organization of objects."""

    # Generate random selection of objects [((x_clone_0), (x_clone_1)), (y_clone_0), ...]
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    # Pick random rotation
    rotation = np.random.random() * 2 * math.pi

    # Number of objects = number of sides to polygon.
    n = sum([len(group) for group in objs])
    # print("==== neat_equidist %s, with %x objs" % ('vert' if vert else 'not vert', n))

    # Generate regular polygon points.
    points = polygon_points(n, radius=min([HALF_WIDTH, HALF_LENGTH]), rotation=rotation,
                            cx=CENTER_X, cy=CENTER_Y)
    # print("points:", points)

    # Create poses from names and points.
    poses = {}
    for name, p in zip(itertools.chain(*objs), points):
        poses[name] = gen_pose(name, p[0], p[1], TABLE_HEIGHT)

    # print(poses)
    return poses


def neat_cluster_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """
        Cluster objects into 1-5 evenly spaced groupings on table.
        Choose randomly between polygon and linearly spaced clusters.
        Within a cluster, items are arranged either linearly or in a polygon.
    """

    # Random selection of objects [((x_clone_0), (x_clone_1)), (y_clone_0), ...]
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    # Generate cluster centroids
    n_centroids = len(objs)
    rotation = np.random.random() * 2 * math.pi
    centroids = polygon_points(sides=n_centroids) #, rotation=rotation)  # TODO rotate?

    # Random vertical/horizontal orientation
    do_polygon = True # np.random.random() > 0.5
    print("==== neat_cluster %s" % ('polygons' if do_polygon else 'linear'))

    # Generate points within cluster with specified method
    points = []
    n_centroids += 2
    for (cx, cy), group in zip(centroids, objs):
        if do_polygon:
            group_rot = np.random.random() * 2 * math.pi
            sides = len(group)
            radius = (HALF_LENGTH + HALF_WIDTH) / 2 / n_centroids
            points = points + polygon_points(sides=sides, radius=radius, rotation=group_rot, cx=cx, cy=cy)
        else:
            types_along_x = np.random.random() > 0.5
            objs = [group]
            dx = HALF_WIDTH / n_centroids
            dy = HALF_LENGTH / n_centroids
            points = points + linear_points(objs=objs, cx=cx, cy=cy, dx=dx, dy=dy, types_along_x=types_along_x)

    # Create poses from names and points.
    poses = {}
    for name, p in zip(itertools.chain(*objs), points):
        poses[name] = gen_pose(name, p[0], p[1], TABLE_HEIGHT)

    # print(poses)
    return poses


SCENE_ORGS = {
    'messy': random_poses,
    'neat_linear': neat_linear_poses,
    'neat_cluster': neat_cluster_poses,
    'neat_polygon': neat_polygon_poses,
    'four_cube': random_cube_poses
}


def main(args):
    # Usage example: python scene_generator.py neat_cluster clustered 100
    try:
        scene_org = SCENE_ORGS[sys.argv[1]]
        img_dir = sys.argv[2]
        count = 150 if len(sys.argv) < 4 else int(sys.argv[3])
    except:
        print("Usage: scene_generator.py <%s> <img_dir> <[opt]count>" % '/'.join(SCENE_ORGS.keys()))
        exit()

    print("==== %s SCENE GENERATOR SAVING TO data/%s ====" % (sys.argv[1], img_dir))
    print("Using %s scene generator to generate 0x%x images." % (scene_org.__name__, count))

    rospy.init_node('scene_maker', anonymous=True)

    # Setup GazeboClient
    gc = GazeboClient(obj_mover=scene_org, min_objs=MIN_OBJ, max_objs=MAX_OBJ,
                      fixed_models=FIXED_MODELS)

    # Setup camera for saving images
    ic = ImageSubscriber(img_dir=img_dir, ref_imgs=BLANK_TABLES)
    
    # Add models manually for four_cube
    if scene_org == random_cube_poses:
      gc.models = {}
      for name in CUBES:
        pose = gen_rand_pose(name, -5, -5, 0, 3, 3)
        gc.models[name] = pose
        gc.pub.publish(pose)

    # Loop generating scenes and saving images, with pause to let scene stabilize.
    stabilize_pause = 0.05
    i = 0

    gc.full_reset()
    gc.full_reset()
    rospy.sleep(stabilize_pause*5)
    while (not rospy.is_shutdown()):
        print("i: ", i)
        gc.generate_scene()
        if i > count:
            print("Saved %x images" % i)
            gc.mover_reset()
            exit()

        # Wait for the scene to stabilize
        # while not gc.is_stable():
        #  pass
        rospy.sleep(stabilize_pause)

        # Save image
        img_name = ic.save_image()
        if img_name != "":
            ic.pop_ref()
            ic.add_ref(img_name)
            i += 1

        # Reset scene and wait for objects to leave table
        gc.mover_reset()
        rospy.sleep(stabilize_pause)
        # hack_i = 0
        # while not ic.check_blank() and hack_i < 100:
        #     hack_i += 1

    print("Saved 0x%x images" % count)
    gc.mover_reset()
    exit()


if __name__ == '__main__':
    main(sys.argv)
