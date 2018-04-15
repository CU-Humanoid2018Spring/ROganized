import gym
from gym import spaces
import numpy as np
import tensorflow as tf
import math
import time
from baselines.organized_learner import OrgLearner, load_data
#import roslib
#roslib.load_manifest('roganized_rl')
from baselines.scene_generator import random_poses
from baselines.core import GazeboClient, ImageConverter
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
import cv2
#from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate

import pyquaternion as pq

def rotate_z(theta):
    rot = pq.Quaternion(axis=[0,0,1],angle=theta)
    q = Quaternion()
    q.x = rot[1]
    q.y = rot[2]
    q.z = rot[3]
    q.w = rot[0]
    return q

class OrganizeEnv(gym.Env):
    def __init__(self):
        self.h = 60
        self.w = 60
        self.max_t = 50
        self.action_space = spaces.Box(-1, 1, shape=(5,))
        self.observation_space = spaces.Box(low=np.zeros(shape=[self.w*self.h,]), high=256*np.ones(shape=[self.w*self.h,]))
        self.evaluator = OrgLearner()


        rospy.init_node("rl_env")
        self.gazebo_client = GazeboClient(obj_mover=random_poses, min_objs=4, max_objs=6,
                                          fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun'])
        self.image_sub = ImageConverter()
        self.reset()
 
    def train_org_learner(self):
        data = load_data('./data')
        # test_data = load_data('/home/robert/Documents/ROganized/Project/baselines/baselines/test')
        test_data = data
        test_x = test_data[0]
        test_y = test_data[1]
        p = np.random.permutation(len(test_y))
        epochs = 10
        print('Training Started')
        for i in range(epochs):
            start = time.time()
            loss = self.evaluator.train(data)
            print('Epoch '+str(i)+'/'+str(epochs)+': Loss: '+str(loss)+' Completed In: '+str(time.time()-start)+'s')
        print('Training Complete')
        self.evaluator.save()


    def initialize(self, sess):
        self.sess = tf.get_default_session()
        self.evaluator.initialize(sess)
    def train_learner(self):
        #self.train_org_learner()
        self.evaluator.restore()
        

    # This needs to be connected to our environment to do anything
    # Need not return anything, just needs to execute the action in gazebo
    def __exec_move__(self, x_from, y_from, x_to, y_to, theta_to):
        new_state = ModelState()
        print(x_from)
        print(y_from)
        obs_shape = np.reshape(self.obs, [60,60])
        new_state.model_name = obs_shape[x_from][y_from]
        #new_state.pose = Pose( Point(x_to,y_to, 0.7), Quaternion(0,0,0,0))
        #new_state.pose.orientaion = rotate_z(new_state.pose, 0, 0, theta_to)
        #self.gazebo_client.gazebo_client.set_pose(new_state)
        new_state = ModelState()
        new_state.model_name = obs_shape[x_from][y_from]
        new_state.pose = Pose( Point(x_to,y_to, 0.7),\
                               Quaternion(0,0,0,1))
        new_state.pose.orientation = rotate_z(theta_to)
        self.gazebo_client.set_pose(new_state)


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
        obs_array = np.zeros([60,60], dtype=np.uint8) 
        names = dict()
        for entry in obs_list:
            raw_id = entry[0]
            if raw_id not in names: names[raw_id] = len(names)+1
            proc_id = names[raw_id]
            # SOMETHING WRONG HERE
            #print('('+str(entry[1])+','+str(entry[2])+')')
            x = int(round(100*entry[1])-170)
            y = int(round(100*entry[2])+30)
            if x < 60 and x >= 0 and y >= 0 and y < 60:
                obs_array[x][y] = proc_id
        return np.hstack(obs_array)

    def __get_img__(self):
        # SOMETHING WRONG HERE
        image = self.image_sub.get_rgb()
        if image is None:
            rospy.logwarn('rgb get None, retrying...')
            rospy.sleep(0.01)
            image = self.image_sub.get_rgb()
        print(image.shape)
        image = cv2.resize(image, (320, 240))
        # image = np.zeros([240,320,3])
        return np.array([image])

    def reset(self):
        self.__init_episode__()
        self.t = 0
        self.obs = self.__get_obs__()
        return self.obs

    def step(self, action):
        x_from = int(round(30*(action.item(0))))+29
        y_from = int(round(30*(action.item(1))))+29
        x_to = 0.3*action.item(2)+2.0
        y_to = 0.3*action.item(3)
        theta_to = action.item(4)*math.pi
        self.__exec_move__(x_from, y_from, x_to, y_to, theta_to)
        self.t += 1

        done = self.t > self.max_t
        self.obs = self.__get_obs__()
        img = self.__get_img__()
        r = self.evaluator.predict(img)
        return self.obs, r, done, {}
