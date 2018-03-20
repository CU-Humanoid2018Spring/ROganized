#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spawn_messy_objects import save_img

roslib.load_manifest('roganized_gazebo')

thresh_time = rospy.Duration(5)
bridge = CvBridge()


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
