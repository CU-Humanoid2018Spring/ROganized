#!/usr/bin/env python

# Helper functions for spawning neat and messy scenes.

from __future__ import print_function

import roslib
import sys
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from collections import Counter

from utils import ImageSubscriber, GazeboClient, gen_pose, gen_rand_pose

roslib.load_manifest('roganized_rl')

models = ModelStates()
fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun']
all_objects = sorted(['wood_cube_5cm',
                      'cricket_ball',
                      'green_hammer',
                      'pink_beer',
                      'demo_cube'])
MIN_OBJ = 4
MAX_OBJ = 6
# TODO: all-caps the constants below
center_x = 2
center_y = .08
height = 0.72
dx = .6
dy = .6
blank_tables = ["blank-table.png", "blank-table-2.png"]


def random_objects(n, selection=all_objects):
    """Return n random sorted model names from the available desk objTwistects. """
    objs_i = np.random.choice(range(len(selection)), size=n, replace=True)
    objs_i.sort()
    return [all_objects[i] for i in objs_i]


def random_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Return dictionary of {name: pos} for publishing. """
    # TODO: Get static table orientations once, including height.
    poses = {}
    objs = random_objects(np.random.randint(low=mincount, high=maxcount + 1))
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
        poses[name] = gen_rand_pose(name, center_x, center_y, height, dx-0.1, dy-0.1)
    return poses


def neat_linear_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Place objects in vertical or horizontal lines, one type of object per line."""
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.
    print("==== neat_linear %s" % 'vert' if vert else 'not vert')
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount + 1)))  # objs = {"name": count}
    # TODO: shuffle obj-type order

    ### Determine pose per object, per line
    center_1, center_2 = (center_x, center_y) if vert else (center_y, center_x)
    d1 = dx if vert else dy
    d2 = dy if vert else dx

    # Spacing between lines, buffered from table edge
    line_spacing = 2 * d1 / (2 + len(objs))
    line_poses = np.arange(-d1 + center_1, d1 + center_1, line_spacing)[1:-1]

    for line_i, (name, count) in enumerate(objs.items()):

        # Position of this line of objects
        line_pos = line_poses[line_i]

        # Spacing between objects along this line, buffered from table edge
        obj_spacing = 2 * d2 / (2 + count)
        obj_poses = np.arange(-d2 + center_2, d2 + center_2, obj_spacing)[1:-1]

        for obj_i in range(count):
            # Clone's name
            obj_name = name + "_clone_" + str(obj_i)

            # Spacing of clone along the line
            obj_pos = obj_poses[obj_i]

            # Generate pose and save to dict.
            poses[obj_name] = gen_pose(name=obj_name,
                                       x=line_pos if vert else obj_pos,
                                       y=obj_pos if vert else line_pos,
                                       z=height)
    print(poses)
    return poses


def neat_equidist_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Generate a 2D linearly spaced organization of objects."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount + 1)))
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.

    # Determine dimension orientation
    d1 = dx if vert else dy
    d2 = dy if vert else dx

    n = sum([count for count in objs.values()])
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

    points = [(center_x + mx * stepx, center_y + my * stepy) for (mx, my) in multiples]

    print("points:")
    for p in points:
        print(p)

    for (name, _), p in zip(objs.items(), points):
        name += "_clone_" + str(n)
        poses[name] = gen_pose(name, p[0], p[1], height)

    print(poses)
    return poses


def neat_cluster_poses(mincount=MIN_OBJ, maxcount=MAX_OBJ):
    """Cluster objects into 1-5 evenly spaced groupings on table."""
    poses = {}
    objs = Counter(random_objects(np.random.randint(low=mincount, high=maxcount + 1)))
    vert = np.random.random() > 0.5  # Randomly pick whether to position vertically or horizontally.

    # Determine dimension orientation
    d1 = dx if vert else dy
    d2 = dy if vert else dx

    clusters = len(objs)
    print("==== neat_equidist %s, with %x object types" % ('vert' if vert else 'not vert', clusters))

    # Case match number of objects to list of positions
    if clusters == 1:
        step1 = 2 * d1 / 3
        step2 = 2 * d2 / 3
        multiples = [(1, 1)]  # center
    if clusters == 2:
        step1 = 0
        step2 = 2 * d2 / 4
        multiples = [(-1, 1), (1, 1)]  # line
    if clusters == 3:
        step1 = 2 * d1 / 3
        step2 = 2 * d2 / 4
        multiples = [(1, -1), (-1, 1), (1, 1)]  # central triangle
    if clusters == 4:
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

    # Scale multiples to fit table
    multiples = [(0.5 * a, 0.5 * b) for a, b in multiples]

    # Assign spacing multiples and step size according to vert value
    if not vert:
        multiples = [(mx, my) for my, mx in multiples]
    stepx, stepy = (step1, step2) if vert else (step2, step1)

    points = [(center_x + mx * stepx, center_y + my * stepy) for (mx, my) in multiples]

    print("points:")
    for p in points:
        print(p)

    for (name, _), p in zip(objs.items(), points):
        name += "_clone_" + str(clusters)
        poses[name] = gen_pose(name, p[0], p[1], height)

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
                      fixed_models=fixed_models)

    # Setup camera for saving images
    ic = ImageSubscriber(img_dir=img_dir, ref_imgs=blank_tables)

    # Loop generating scenes and saving images, with pause to let scene stabilize.
    stabilize_pause = 0.05
    i = 0
    while (not rospy.is_shutdown()):
        gc.generate_scene()
        if i > count:
            print("Saved %x images" % i)
            exit()
        # Wait for the scene to stabilize
        # while not gc.is_stable():
        #  pass
        # rospy.sleep(stabilize_pause)
        # Save image
        img_name = ic.save_image()
        if img_name != "":
            ic.add_ref(img_name)
            i += 1
        gc.mover_reset()
    print("Saved 0x%x images" % count)
    gc.cancel()
    ic.cancel()


if __name__ == '__main__':
    main(sys.argv)
