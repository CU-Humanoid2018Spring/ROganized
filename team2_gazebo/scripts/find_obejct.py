#!/usr/bin/env python
from __future__ import print_function

import roslib
import actionlib
roslib.load_manifest('team2_gazebo')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from control_msgs.msg import PointHeadActionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from cv_bridge import CvBridge, CvBridgeError
import  numpy as np

class ImageConverter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/team2/output_image",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ###########################################################################
    # OPENCV PIPELINE START HERE
    (rows,cols,channels) = cv_image.shape

    # UNCOMMENT IF YOU WANT USE OPENCV VIEWER
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    # OPENCV PIPELINE END HERE
    ###########################################################################

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("/head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

def main(args):
  ic = ImageConverter()
  rospy.init_node('image_converter', anonymous=True)
  head = PointHeadClient()
  try:
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
      x = 0.5 + np.random.randn() * 0.5
      y = np.random.randn() * 0.15
      z = 0.0
      rospy.loginfo("x, y, z = "+str(x)+","+str(y)+","+str(z))
      head.look_at(x,y,1, "base_link")
      rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
