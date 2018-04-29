from geometry_msgs.msg import PoseStamped ,Pose, Point, Quaternion, TransformStamped
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_matrix
import tf
import rospy
import numpy as np

def frame_transformation(parent_frame_id, child_frame_id):
    """ Returns geometry_msgs.msg.Pose
    Retrive the relative transformation pose from parent to child
    """
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform(parent_frame_id, child_frame_id, rospy.Time(0))
            return Pose(Point(trans[0],trans[1],trans[2]), Quaternion(rot[0],rot[1],rot[2],rot[3]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()


def transform_pose(source_pose, source_frame, target_frame):
    """ Returns geometry_msgs.msg.Pose
    Transform the pose into another frame's perspective
    """
    pose_stamped = PoseStamped()
    pose_stamped.pose = source_pose
    pose_stamped.header.frame_id = source_frame
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            target_pose = listener.transformPose(target_frame=target_frame,ps=pose_stamped)
            return target_pose.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

def relative_pose(pose1, pose2):
    """ Returns geometry_msgs.msg.Pose
    Compute relative pose from pose1 to pose2
    pose1 and pose2 are two poses measured from world frame
    """
    # Relative translation: p1 = R^0_1 * p0 + d^0_1
    q1_inv = quaternion_inverse([pose1.orientation.x, pose1.orientation.y,\
                                 pose1.orientation.z, pose1.orientation.w])
    T1_inv = quaternion_matrix(q1_inv)
    T1_inv[:3,3] = [-pose1.position.x, -pose1.position.y, -pose1.position.z]
    p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z, 1.0])
    p_res = T1_inv.dot(p2)

    # Relative rotation
    q2 = [pose2.orientation.x,pose2.orientation.y,pose2.orientation.y,pose2.orientation.w]
    q_res = quaternion_multiply(q1_inv, q2)

    result = Pose(Point(p_res[0],p_res[1], p_res[2]), Quaternion(q_res[0], q_res[1], q_res[2], q_res[3]))
    return result
