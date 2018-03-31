#!/usr/bin/env python

# Helper functions for spawning neat and messy scenes.

from __future__ import print_function

import roslib
import sys
import random
import rospy
import itertools
import numpy as np
from gazebo_msgs.msg import ModelStates
from collections import Counter

from utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

roslib.load_manifest('roganized_rl')

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
DX = .58
DY = .58
BLANK_TABLES = ["blank-table.png", "blank-table-2.png", "blank-table-3.png"]


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
            poses[name] = gen_rand_pose(name, CENTER_X, CENTER_Y, TABLE_HEIGHT, DX-.1, DY-.1)
    return poses


def neat_linear_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Place objects in vertical or horizontal lines, one type of object per line."""
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    print("==== neat_linear %s" % ('vert' if vert else 'not vert'))
    poses = {}
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    ### Determine pose per object, per line
    center_1, center_2 = (CENTER_X, CENTER_Y) if vert else (CENTER_Y, CENTER_X)
    d1 = DX if vert else DY
    d2 = DY if vert else DX

    # Spacing between lines, buffered from table edge
    line_spacing = 2 * d1 / (2 + len(objs))
    line_poses = np.arange(-d1 + center_1, d1 + center_1, line_spacing)[1:-1]
    print('line_poses: ', line_poses)

    for line_i, group in enumerate(objs):
        count = len(group)

        # Position of this line of objects
        line_pos = line_poses[line_i]

        # Spacing between objects along this line, buffered from table edge
        obj_spacing = 2 * d2 / (2 + count)
        obj_poses = np.arange(-d2 + center_2, d2 + center_2, obj_spacing)[1:-1]
        print(group[0], obj_poses)

        for obj_i, obj_name in enumerate(group):
            # Spacing of clone along the line
            obj_pos = obj_poses[obj_i]

            # Generate pose and save to dict.
            poses[obj_name] = gen_pose(name=obj_name,
                                       x=line_pos if vert else obj_pos,
                                       y=obj_pos if vert else line_pos,
                                       z=TABLE_HEIGHT)
    # print(poses)
    return poses


def neat_equidist_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a 2D linearly spaced organization of objects."""
    poses = {}
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    # Determine dimension orientation
    d1 = DX if vert else DY
    d2 = DY if vert else DX

    n = sum([len(group) for group in objs]) # number of objects total
    print("==== neat_equidist %s, with %x objs" % ('vert' if vert else 'not vert', n))

    # Case match number of objects to list of positions
    if n == 4:
        step1 = 2 * d1 / 4
        step2 = 2 * d2 / 4
        multiples = [(-1, -1), (1, -1),
                     (-1, 1), (1, 1)]  # central square
    elif n == 5:
        step1 = 2 * d1 / 4
        step2 = 2 * d2 / 5
        multiples = [(-1, -1), (1, -1),
                     (0, 0),
                     (-1, 2), (1, 2)]  # box with center
    else:
        step1 = 2 * d1 / 5
        step2 = 2 * d2 / 5
        multiples = [(-1, -1), (-1, 0), (-1, 2),
                     (0, -1), (0, 0), (0, 2),
                     (2, -1), (2, 0), (2, 2)]  # 2x3

    # Scale multiples to fit table
    multiples = [(0.5 * a, 0.5 * b) for a, b in multiples]

    # Assign spacing multiples and step size according to vert value
    if not vert:
        multiples = [(mx, my) for my, mx in multiples]
    stepx, stepy = (step1, step2) if vert else (step2, step1)

    points = [(CENTER_X + mx * stepx, CENTER_Y + my * stepy) for (mx, my) in multiples]

    print("points:")
    for p in points:
        print(p)

    print("***chaining names: ", list(itertools.chain(*objs)))
    for name, p in zip(itertools.chain(*objs), points):
        poses[name] = gen_pose(name, p[0], p[1], TABLE_HEIGHT)

    # print(poses)
    return poses


def neat_cluster_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Cluster objects into 1-5 evenly spaced groupings on table."""
    poses = {}
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))

    # Determine dimension orientation
    d1 = DX if vert else DY
    d2 = DY if vert else DX

    clusters = len(objs)
    print("==== neat_cluster %s, with %x object types" % ('vert' if vert else 'not vert', clusters))

    # Case match number of objects to list of cluster centers
    if clusters == 1:
        step1 = 2 * d1 / 3
        step2 = 2 * d2 / 3
        multiples = [(1, 1)]  # center
    elif clusters == 2:
        step1 = 0
        step2 = 2 * d2 / 4
        multiples = [(-0.5, .5), (.5, .5)]  # line
    elif clusters == 3:
        step1 = 2 * d1 / 3
        step2 = 2 * d2 / 4
        multiples = [(.5, -.5), (-.5, .5), (.5, .5)]  # central triangle
    elif clusters == 4:
        step1 = 2 * d1 / 4
        step2 = 2 * d2 / 4
        multiples = [(-1, -1), (1, -1),
                     (-1, 1), (1, 1)]  # central square
    elif clusters == 5:
        step1 = 2 * d1 / 4
        step2 = 2 * d2 / 5
        multiples = [(-1, -1), (1, -1),
                     (0, 0),
                     (-1, 2), (1, 2)]  # box with center
    else:
        step1 = 2 * d1 / 5
        step2 = 2 * d2 / 5
        multiples = [(-1, -1), (-1, 0), (-1, 2),
                     (0, -1), (0, 0), (0, 2),
                     (2, -1), (2, 0), (2, 2)]  # 2x3

    # Assign spacing multiples and step size according to vert value
    if not vert:
        multiples = [(mx, my) for my, mx in multiples]
    stepx, stepy = (step1, step2) if vert else (step2, step1)

    centroids = [(CENTER_X + mx * stepx, CENTER_Y + my * stepy) for (mx, my) in multiples]

    print("points:")
    for p in centroids:
        print(p)

    dx = DX / clusters
    dy = DY / clusters
    for group, (centroid_x, centroid_y) in zip(objs, centroids):
        for name in group:
            poses[name] = gen_rand_pose(name, centroid_x, centroid_y, TABLE_HEIGHT, dx, dy)

    print(poses)
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

    # Loop generating scenes and saving images, with pause to let scene stabilize.
    stabilize_pause = 0.2
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
        rospy.sleep(stabilize_pause*3)
        # hack_i = 0
        # while not ic.check_blank() and hack_i < 100:
        #     hack_i += 1

    print("Saved 0x%x images" % count)
    gc.mover_reset()
    exit()


if __name__ == '__main__':
    main(sys.argv)
