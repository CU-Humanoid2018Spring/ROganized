#!/usr/bin/python
from graspit_commander import GraspitCommander
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import rospy

# Wait several seconds for demo
def count_seconds(sec):
	for i in range(sec,-1,-1):
		time.sleep(1)
		rospy.loginfo(str(i)+" seconds")

def test_grasp(gc, n):
	for i in range(n):
		gc.autoOpen()
		time.sleep(1)
		gc.autoGrasp()
		time.sleep(1)
	rospy.loginfo("grasp complete: "+str(gc.dynamicAutoGraspComplete()))


def demo():
	rospy.init_node('hw3_demo', anonymous=True)
	gc = GraspitCommander()
	gc.clearWorld()

	# Load in the fetch gripper and fully extend out the gripper
	rospy.loginfo("import the fetch gripper...");
	gripper = gc.importRobot('fetch_gripper')
	rospy.loginfo("extend out the fetch gripper..."); count_seconds(2)
	gc.autoOpen()

	# Load the a mug
	rospy.loginfo("import mug");
	mug = gc.importGraspableBody('mug')

	# Place the robot and the object
	rospy.loginfo("manual grasp pose"); count_seconds(3)
	gripper_pose = Pose( position=Point(-0.0704542484271,-0.201466631816, 0.0016985854928),
			orientation=Quaternion(0.0108604258991, 0.0360529356943, 0.675958273869, 0.735977342698))
	gc.setRobotPose(gripper_pose,id=0)
	test_grasp(gc, 2)

	# Start plalnning (succeed if contact is reached)
	rospy.loginfo("manual grasp pose"); count_seconds(3)
	gripper_pose = Pose( position=Point(0.06,-0.20,0), orientation=Quaternion(0,0,1.1,1))
	gc.setRobotPose(gripper_pose,id=0)

	attempts = 1
	while ( not gc.dynamicAutoGraspComplete() and attempts < 5):
		rospy.loginfo("graspit planing attempt "+str(attempts))
		attempts += 1
		gc.planGrasps()
		test_grasp(gc, 2)
	if (gc.dynamicAutoGraspComplete()):
		rospy.loginfo("graspit done")
	else:
		rospy.loginfo("graspit failed")

	# Show Grasp Pose
	gripper_pose = gc.getRobot().robot.pose
	rospy.loginfo("current gripper pose:\n"+str(gripper_pose))


if __name__ == "__main__":
	try:
		demo()
	except rospy.ROSInterruptException:
		pass

#position: 
#  x: -0.0783711574588
#  y: -0.199957736676
#  z: -0.00277594091231
#orientation: 
#  x: -0.748050585377
#  y: -0.663268104222
#  z: -0.00135911541785
#  w: 0.02222378105
#
#
#position: 
#  x: -0.129967714482
#  y: -0.124904377135
#  z: -0.114554789371
#orientation: 
#  x: -0.827992077582
#  y: -0.410221547237
#  z: -0.333610499856
#  w: 0.186685393186
#
#position: 
#  x: -0.0993304524196
#  y: -0.192954375263
#  z: 0.00262800886572
#orientation: 
#  x: 0.00929690737972
#  y: 0.0279539832699
#  z: 0.613405008896
#  w: 0.789218878001
#
#position: 
#  x: -0.146068761144
#  y: -0.154331083915
#  z: 0.0548229387233
#orientation: 
#  x: -0.0683168182774
#  y: 0.181889792833
#  z: 0.451810515838
#  w: 0.87069866968

#position: 
#  x: -0.0704542484271
#  y: -0.201466631816
#  z: 0.0016985854928
#orientation: 
#  x: 0.0108604258991
#  y: 0.0360529356943
#  z: 0.675958273869
#  w: 0.735977342698
