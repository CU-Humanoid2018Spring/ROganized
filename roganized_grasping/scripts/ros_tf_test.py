#!/usr/bin/env python

from roganized_grasping.transform import frame_transformation, transform_pose
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
if __name__ == '__main__':
    rospy.init_node('get_tf')
    source = 'odom'
    target = 'base_link'

    print '===================================================='
    print 'frame_transformation({}, {})'.format(source, target)
    print frame_transformation('odom', 'base_link')

    print '===================================================='
    pose = Pose(Point(0.4, -0.1, 0.35), Quaternion(0,0,0,1))
    print 'source_pose'
    print pose
    print 'transform_pose(source_pose, {}, {})'.format(source, target)
    print transform_pose(pose, 'odom', 'base_link')
    print '===================================================='
