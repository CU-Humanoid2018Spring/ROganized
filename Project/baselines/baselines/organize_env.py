import gym
from gym import spaces
import numpy as np
import tensorflow as tf
import math
from baselines.organized_learner import OrgLearner
import roslib
roslib.load_manifest('roganized_rl')
from baselines.scene_generator import random_poses
from baselines.ros_utils import GazeboClient
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate

class OrganizeEnv(gym.Env):
    def __init__(self):
        self.h = 144
        self.w = 256
        self.max_t = 50
        self.action_space = spaces.Box(-1, 1, shape=(5,))
        self.observation_space = spaces.Box(low=np.zeros(shape=[self.w, self.h, 3]), high=np.ones(shape=[self.w, self.h, 3]))
        self.evaluator = OrgLearner()
        self.evaluator.restore()
        self.gazebo_client = GazeboClient(obj_mover=random_poses, min_objs=4, max_objs=6,
                                          fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun'])

    # This needs to be connected to our environment to do anything
    # Need not return anything, just needs to execute the action in gazebo
    def __exec_move__(self, x_from, y_from, x_to, y_to, theta_to):
        new_state = ModelState()
        new_state.model_name = self.obs[int(round(x_from))][int(round(y_from))]
        new_state.pose = Pose( Point(x_to,y_to, 0.7), Quaternion(0,0,0,0))
        new_state.pose = rotate(new_state.pose, 0, 0, theta_to)
        self.gazebo_client.gazebo_client.set_pose(new_state)

    # This needs to be connected to our environment to do anything
    # Need not return anything, juse needs to randomly initialize the gazebo env
    def __init_episode__(self):
        self.gazebo_client.mover_reset()
        self.gazebo_client.generate_scene()
        rospy.sleep(0.1)

    # This needs to be connected to our environment to do anything
    # Should return a numpy array of dimensions [self.w, self.h, 3]
    def __get_obs__(self):
        obs_list = self.gazebo_client.get_rl_state()
        obs_array = np.zeros([60,60]) 
        names = dict()
        for entry in obs_list:
            raw_id = entry[0]
            if raw_id not in names: names[raw_id] = len(names)+1
            proc_id = names[raw_id]
            x = int(round(entry[1]-200))
            y = int(round(entry[2]))
            obs_array[x][y] = proc_id
        return obs_array

    def __get_img__(self):
        image = self.image_sub.get_rgb()
        image = cv2.resize(image, (320, 240))
        return image

    def reset(self):
        self.__init_episode__()
        self.t = 0
        self.obs = self.__get_obs__()
        return self.obs

    def step(self, action):
        x_from = action.item(0)+1
        y_from = action.item(1)+1
        x_to = action.item(2)+1
        y_to = action.item(3)+1
        theta_to = action.item(4)*math.pi
        self.__exec_move__(x_from, y_from, x_to, y_to, theta_to)
        self.t += 1

        done = self.t > self.max_t
        self.obs = self.__get_obs__()
        img = self.__get_img__()
        r = self.evaluator.predict(img)
        return self.obs, r, done, {}
