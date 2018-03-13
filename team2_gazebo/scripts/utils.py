#!/usr/bin/env python
import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface,PickPlaceInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Pick and place objects using the manipulation stack.
class Robot:
  def __init__(self):
    # Create move group interface for a fetch robot
    self.move_group = MoveGroupInterface('arm_with_torso', 'base_link')
    self.gripper_frame = 'wrist_roll_link'
    self.gripper_pose_stamped = PoseStamped()
    self.gripper_pose_stamped.header.frame_id = 'base_link'

    self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)

  def moveto(self, pose):
    self.gripper_pose_stamped.header.stamp = rospy.Time.now()
    self.gripper_pose_stamped.pose = pose
    self.move_group.moveToPose(self.gripper_pose_stamped, self.gripper_frame)
    result = self.move_group.get_move_action().get_result()

    if result:
      # Checking the MoveItErrorCode
      if result.error_code.val == MoveItErrorCodes.SUCCESS:
        rospy.loginfo("Hello there!")
      else:
        # If you get to this point please search for:
        # moveit_msgs/MoveItErrorCodes.msg
        rospy.logerr("Arm goal in state: %s",
                      self.move_group.get_move_action().get_state())
    else:
      rospy.logerr("MoveIt! failure no result returned.")

  def cancel(self):
    self.move_group.get_move_action().cancel_all_goals()

from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist
class Gazebo:
  def __init__(self):
    # gazebo/ModelStates stores object in a list and contains twist
    # since we do not need twist and desire a constant access to the obejct,
    # we use a dictionary to store the objects
    self.models = None
    self.sub = rospy.Subscriber('/gazebo/model_states',
                                ModelStates, self.model_callback)
    self.pub = rospy.Publisher('/gazebo/set_model_state',
                               ModelState, queue_size = 10)
    self.fixed_models = {'table1', 'table2', 'fetch', 'ground_plane', 'camera'}
    for i in range(3,0,-1):
      rospy.loginfo('waiting for callbacks... %i seconds',i)
      rospy.sleep(1)
    
    self.reset()

  def model_callback(self, msg):
    if self.models is None:
      self.models={}
      for i, name in enumerate(msg.name):
	if name in self.fixed_models:
	  pass
	else:
          rospy.loginfo("add model name: %s", name)
          self.models[name] = msg.pose[i]
    else:
      for i, name in enumerate(msg.name):
        if name in self.models:
	  self.models[name] = msg.pose[i]
	elif name in self.fixed_models:
	  pass
	else:
	  rospy.logerr("model name %s does not exist", name)

  def reset(self):
    default_state = ModelState()
    if self.models is None:
      rospy.logerr("models is None")
      return
    x = 0.0
    for name, pose in self.models.items():
      default_state.model_name = name
      default_state.pose.orientation = Quaternion(0,0,0,0)
      default_state.pose.position = Point(x, -1.5, 0.0)
      x += 0.5
      self.pub.publish(default_state)
    default_state.model_name='table1'
    default_state.pose.position = Point(0.8,0.35,-0.08)
    default_state.pose.orientation = Quaternion(0,0,0,0)
    self.pub.publish(default_state)
    default_state.model_name='table2'
    default_state.pose.position = Point(0.8,-0.35,-0.08)
    default_state.pose.orientation = Quaternion(0,0,0,0)
    self.pub.publish(default_state)

  def relocate(self, name, pose):
    state = ModelState()
    state.model_name = name
    state.pose= pose
    self.pub.publish(state)

  def get_pose(self, name):
    return self.models[name]

class RL:
  def __init__(self):
    self.locations = [(0.6, 0.2), (0.6,-0.2)]
    self.index = 3
  def action(self):
    self.index = (self.index+1)%2
    return {'name': 'cricket_ball','x': self.locations[self.index][0],
            'y': self.locations[self.index][1]}

if __name__ == '__main__':
  rospy.init_node("fetch")
  fetch = Robot()
  gazebo = Gazebo()
  rl = RL()
  gazebo.reset()
  gazebo.relocate('cricket_ball',Pose(Point(0.6,0.2,1.0),Quaternion(0,0,0,0)))
  #gripper_poses = [Pose(Point(0.042, 0.384, 1.826),
  #                      Quaternion(0.173, -0.693, -0.242, 0.657)),
  #                 Pose(Point(0.047, 0.545, 1.822),
  #                      Quaternion(-0.274, -0.701, 0.173, 0.635))]
  downward = Quaternion(0.0,1.57,0.0,1.57)
  gripper_pose = Pose(Point(0.7, 0.3, 1.0),downward)
  while not rospy.is_shutdown():
    # request action from RL model
    action = rl.action()
    print action
    # grasp target
    target_pose = gazebo.get_pose(action['name'])
    target_pose.orientation = downward
    target_pose.position.z += 0.2
    fetch.moveto(target_pose)
    # place target
    gripper_pose.position.x = action['x']
    gripper_pose.position.y = action['y']
    fetch.moveto(gripper_pose)
  fetch.cancel()
