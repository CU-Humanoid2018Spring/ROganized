#!/usr/bin/env python
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

roslib.load_manifest('team2_gazebo')

'''
Spawn and save random object configurations to 'Humanoid-Team2/messy_imgs'.
'''

models = ModelStates()
static_objs = ['table1', 'table2', 'camera']
all_objects = ['bowl',
               'coke_can',
               'cordless_drill',
               'cricket_ball',
               'hammer',
               'marble_1_5cm',
               'plastic_cup',
               'parrot_bebop_2']
MIN_OBJ = 4
MAX_OBJ = 9
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

def model_callback(msg): # TODO: Previous models are unused atm, possible to use to generate organized version?
    # TODO: Get static table orientations once.
    # TODO: get table height, table_z is -0.0002 inside msg.pos

    # Center objects for placement
    center_x, center_y = 2, 0

    # Clear previous objects
    for o in msg.name:
        # Skip objects we want to keep in the scene, e.g. tables.
        if o in static_objs:
            continue
        mover = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        random_pose = gen_rand_pose(o, -5, -5, 0, 3, 3)
        mover.publish(random_pose)

    objs = random_objects(np.random.randint(low=MIN_OBJ, high=MAX_OBJ))
    prev = all_objects[0]
    i = 1
    for o in objs:
        # Track which object clone to reference.
        if o == prev:
            i += 1
        else:
            prev = o
            i = 0
        o += "_clone_" + str(i)
        
        # Generate a pos on the table and publish.
        mover = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        random_pose = gen_rand_pose(o, center_x, center_y, height, dx, dy)
        mover.publish(random_pose)


def save_img(img, subdir, prefix='scene', suffix='.png'):
    cwd = os.getcwd()
    if os.path.basename(cwd) == 'Humanoid-Team2':
        path = os.path.join(os.getcwd(), subdir)
    elif os.path.basename(cwd) == 'team2_ws':
        path = os.path.join(os.getcwd(), "src/Humanoid-Team2", subdir)
    else:
        path = cwd
    if not os.path.exists(path):
        os.makedirs(path)
        print("Making path to ", path)
    n = len(os.listdir(path))
    img_path = os.path.join(path, prefix + str(n) + suffix)
    cv2.imwrite(img_path, img)
    print("saved image to: ", img_path)


class ImageConverter:
    """Subscribe to image feed and publish to img_dir/."""

    def __init__(self, img_dir='messy_imgs'):
        self.image_pub = rospy.Publisher("/team2/output_image", Image, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.img_dir = img_dir


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            save_img(cv_image, self.img_dir)
            cv2.imshow('image', cv_image)
            cv2.waitKey(2)

        except CvBridgeError as e:
            print(e)


def main(args):

    rospy.init_node('model_spawner', anonymous=True)
    # model_callback initiates scene generation
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=1)

    # Setup camera for saving images
    ic = ImageConverter(img_dir='messy_imgs')

    r = rospy.Rate(20) # 1Hz
    while not rospy.is_shutdown():
        # random_pose.pose.position.x = 1 + np.random.uniform(-0.25,0.25)
        # random_pose.pose.position.y = 0.01 + np.random.uniform(-0.75,0.75)
        # cup_mover.publish(random_pose)
        r.sleep()

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
