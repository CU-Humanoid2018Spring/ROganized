import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point , Quaternion, Twist
from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate

def move_base(x,y,theta):
    rospy.init_node('move_base')
    #pub = rospy.Publisher('/move_base_simple/goal', PoseStamped,\
    #                      queue_size=10)
    #pose = PoseStamped()
    #pose.header.frame_id = 'base_link'
    #pose.pose = Pose(Point(x, y, 0.0), Quaternion(0,0,0,1))
    #pose.pose = rotate(pose.pose, 0, 0, theta)
    twist = Twist()
    twist.linear.x = 1
    twist.linear.y = twist.linear.z = 0
    twist.angular.x = twist.angular.y = twist.angular.z = 0
    pub = rospy.Publisher('/base_controller/command', Twist, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(twist)
        rospy.sleep(0.1)


