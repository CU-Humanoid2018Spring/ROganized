#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import numpy as np
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

roslib.load_manifest('team2_gazebo')

thresh_time = rospy.Duration(5)
bridge = CvBridge()


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

def callback_maker(start_time):
    def callback(data):
        print("callback")
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow('image', cv_image)

            take_pic = raw_input("Enter 'y' to save picture.")
            if take_pic == 'y':
                subdir = raw_input('Enter subdirectory: ')

                save_img(cv_image, subdir)
                cv2.destroyAllWindows()
        except CvBridgeError as e:
            print(e)
    return callback


# Subscribe to top-down Rviz image feed
def main(args):
    rospy.init_node('image_saver', anonymous=True)

    start_time = rospy.Time.now()

    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_maker(start_time), queue_size=1)

    r = rospy.Rate(.5) # 1Hz
    while not rospy.is_shutdown():
        r.sleep()



if __name__ == '__main__':
    main(sys.argv)
