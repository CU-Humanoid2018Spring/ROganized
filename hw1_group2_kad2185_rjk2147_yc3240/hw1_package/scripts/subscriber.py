#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": heard %s", data.data)
    file.write(str(data.data)+'\n')
    
def subscriber():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("/hw1_time", String, callback)
    rospy.spin()

if __name__ == '__main__':
    file = open('hw1_output.txt','wb')
    subscriber()
    file.close()
