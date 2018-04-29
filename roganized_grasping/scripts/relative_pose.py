#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_multiply
from roganized_gazebo.table_manager import TableManager
from roganized_grasping.transform import relative_pose

'''
def _inverse_quaternion(q):
    q_inv = Quaternion()
    q_inv.x = -q.x
    q_inv.y = -q.y
    q_inv.z = -q.z
    q_inv.w =  q.w
    return q_inv

def relative_pose(pose1, pose2):
    result = Pose()
    inv = _inverse_quaternion(pose1.orientation)
    q1_inv = [inv.x, inv.y, inv.z, inv.w]
    q2 = [pose2.orientation.x,pose2.orientation.y,pose2.orientation.y,pose2.orientation.w]
    q_res = quaternion_multiply(q1_inv, q2)
    result.orientation = Quaternion(q_res[0], q_res[1], q_res[2], q_res[3])
    result.position.x = pose2.position.x - pose1.position.x
    result.position.y = pose2.position.y - pose1.position.y
    result.position.z = pose2.position.z - pose1.position.z
    return result
'''

if __name__ == '__main__':
    table = TableManager()
    for i in range(3):
        rospy.sleep(1)
    robot_pose = table.models['fetch']
    cube_pose = table.models['cube_0']
    print '===================================='
    print 'robot_pose'
    print robot_pose
    print '===================================='
    print 'cube_pose'
    print cube_pose
    print '===================================='
    print 'relative_pose'
    print relative_pose(robot_pose, cube_pose)
    #print _inverse_quaternion(robot_pose.orientation)
