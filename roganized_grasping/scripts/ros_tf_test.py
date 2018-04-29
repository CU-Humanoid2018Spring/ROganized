#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import tf
import rospy
from tf import TransformListener

def get_tf_pose(parent_frame_id, child_frame_id):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(parent_frame_id, child_frame_id, rospy.Time(0))
            return Pose(Point(trans[0],trans[1],trans[2]), Quaternion(rot[0],rot[1],rot[2],rot[3]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('get_tf')
    print get_tf_pose('odom', 'base_link')
    print 'hello'
