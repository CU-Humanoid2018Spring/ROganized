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
all_objects = ['bowl',
               'coke_can',
               'cricket_ball',
               'drill',
               'hammer',
               'marble_1_5cm',
               'plastic_cup',
               'parrot_bebop_2']


def random_objects(n, selection=all_objects):
    """Pick n random object models by name from the available desk objects. """
    objs_i = np.random.choice(range(len(selection)), size=n, replace=True)
    return [all_objects[i] for i in objs_i]

# This example shows how to extract model info from gazebo topics
def model_callback(msg):
    models = msg  # TODO: Previous models are unused atm, possible to use to generate organized version?
    table1_x, table1_y, table1_z, _ = msg.pose[msg.find('table1')]  # Get pos values of table1 from message
    table2_x, table2_y, table2_z, _ = msg.pose[msg.find('table2')]  # Get pos values of table1 from message

    print("table1 located at: ", table1_x, table1_y, table1_z)
    print("table2 located at: ", table2_x, table2_y, table2_z)

    # height = table1_z  # table_z is something like -0.0002 from the msg pos
    height = 0.75  # TODO: get table height
    center_x, center_y = (table1_x+table2_x)/1, (table1_y+table2_y)/2
    dx = dy = max( abs(table2_y - table1_y)/2, abs(table2_x - table1_x)/2 )  # TODO: are the tables square?

    objs = random_objects(np.random.randint(low=4, high=10))
    for o in objs:
        mover = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        random_pose = ModelState()
        random_pose.model_name = o
        random_pose.pose.orientation = Quaternion(1, .01, 75, 0)
        random_pose.pose.position.z = height  # height of table
        random_pose.pose.position.x = center_x + np.random.uniform(-dx, dx)
        random_pose.pose.position.y = center_y + np.random.uniform(-dy, dy)
        mover.publish(random_pose)


def shuffle_objects(msg):
    """Shuffle objects on table. No return."""
    table_height = msg.pose[msg.find('table')][3]  # Get height of table from message
    print("height of table: ", table_height)

    for i, model_name in enumerate(msg.name):
        mover = rospy.Publisher('/gazebo/set_model_state',ModelState, queue_size = 10)
        random_pose = ModelState()
        random_pose.model_name = model_name
        random_pose.pose.orientation = Quaternion(0,0,0,0)
        random_pose.pose.position.z = table_height  # TODO: check this height is ok

    pass


def publish_and_save_new_config():
    """Push changes to object configuration, and take picture."""


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
    """Subscribe to image feed and publish to output_image/."""

    def __init__(self, img_dir='messy_imgs'):
        self.image_pub = rospy.Publisher("/team2/output_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.img_dir = img_dir


    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            save_img(cv_image, self.img_dir)
            cv2.imshow('image', cv_image)
            cv2.waitKey(2000)

        except CvBridgeError as e:
            print(e)


def main(args):

    rospy.init_node('model_spawner', anonymous=True)
    # model_callback initiates scene generation
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=1)

    # Setup camera for saving images
    ic = ImageConverter(img_dir='messy_imgs')

    r = rospy.Rate(.5) # 1Hz
    while not rospy.is_shutdown():
        # random_pose.pose.position.x = 1 + np.random.uniform(-0.25,0.25)
        # random_pose.pose.position.y = 0.01 + np.random.uniform(-0.75,0.75)
        # cup_mover.publish(random_pose)
        r.sleep()

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
