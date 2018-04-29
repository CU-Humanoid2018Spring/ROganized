from geometry_msgs.msg import PoseStamped ,Pose, Point, Quaternion, TransformStamped
import tf
import rospy

def frame_transformation(parent_frame_id, child_frame_id):
    """ Returns geometry_msgs.msgs.Pose
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
    """ Returns geometry_msgs.msgs.Pose
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
