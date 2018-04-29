import gym
from gym import spaces
import numpy as np
from collections import deque
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
        self.last_100_eps = deque(maxlen=100)
        self.last_100_finals = deque(maxlen=100)
        self.ep_rew = 0.0
        self.ep_nb = 0
        self.r = 0
        self.action_space = spaces.Box(-1, 1, shape=(4,))
        self.observation_space = spaces.Box(low=np.zeros(shape=[self.w*self.h,]), high=256*np.ones(shape=[self.w*self.h,]))
        self.evaluator = OrgLearner()


        rospy.init_node("rl_env")
        self.gazebo_client = GazeboClient(obj_mover=random_poses, min_objs=4, max_objs=6,
                                          fixed_models = ['table', 'fetch', 'ground_plane', 'camera', 'sun'])
        self.image_sub = ImageConverter()
        self.reset()
 
    def train_org_learner(self):
        data = load_data('./data/scored_with_poses_30k')
        # valid = load_data('./valid')
        # test_data = load_data('/home/robert/Documents/ROganized/Project/baselines/baselines/test')
        epochs = 10
        print('Training Started')
        for i in range(epochs):
            start = time.time()
            loss = self.evaluator.train(data)
            print('Epoch '+str(i)+'/'+str(epochs)+': Loss: '+str(loss)+' Completed In: '+str(time.time()-start)+'s')
        print('Training Complete')
        i = 0
        # test_x = valid[0]
        # test_y = valid[1]
        # p = np.random.permutation(len(test_y))
        # correct = 0
        # incorrect = 0
        # predictions = []
        # test_y = np.array(test_y)
        # Y = np.transpose(np.expand_dims(test_y, axis=0))
        # print('Testing Started')
        # while i < len(test_x):
        #     x_sample = [test_x[j] for j in p[i:i + 1]]
        #     y_sample = [test_y[j] for j in p[i:i + 1]]
        #     x_batch = np.array(x_sample)
        #     y_batch = np.array(y_sample)
        #     start = time.time()
        #     predictions.append(self.evaluator.predict(x_sample))
        #     #print(str(i)+'/'+str(len(test_y))+': Pred: '+str(predictions[-1])+' Real: '+str(y_sample[0])+' in '+str(time.time()-start)+'s')
        #     pred = int(predictions[-1] >= 0.5)
        #     ans = int(y_sample[0] >= 0.5)
        #     if pred == ans: correct += 1
        #     else: incorrect += 1
        #     i = i + 1
        # print('Valid Accuracy: '+str(float(100.0*correct)/(correct+incorrect)))
        i = 0
        test_x = data[0]
        test_y = data[1]
        p = np.random.permutation(len(test_y))
        correct = 0
        incorrect = 0
        predictions = []
        test_y = np.array(test_y)
        Y = np.transpose(np.expand_dims(test_y, axis=0))
        while i < len(test_x):
            x_sample = [test_x[j] for j in p[i:i + 1]]
            y_sample = [test_y[j] for j in p[i:i + 1]]
            x_batch = np.array(x_sample)
            y_batch = np.array(y_sample)
            start = time.time()
            predictions.append(self.evaluator.predict(x_sample))
            #print(str(i)+'/'+str(len(test_y))+': Pred: '+str(predictions[-1])+' Real: '+str(y_sample[0])+' in '+str(time.time()-start)+'s')
            pred = int(predictions[-1] >= 0.5)
            ans = int(y_sample[0] >= 0.5)
            if pred == ans: correct += 1
            else: incorrect += 1
            i = i + 1
        print('Train Accuracy: '+str(float(100.0*correct)/(correct+incorrect)))

        self.evaluator.save()

    def test(self, data):
        valid = data
        i = 0
        test_x = valid[0]
        test_y = valid[1]
        p = np.random.permutation(len(test_y))
        correct = 0
        incorrect = 0
        predictions = []
        test_y = np.array(test_y)
        Y = np.transpose(np.expand_dims(test_y, axis=0))
        #print('Testing Started')
        while i < len(test_x):
            x_sample = [test_x[j] for j in p[i:i + 1]]
            y_sample = [test_y[j] for j in p[i:i + 1]]
            x_batch = np.array(x_sample)
            y_batch = np.array(y_sample)
            start = time.time()
            predictions.append(self.evaluator.predict(x_sample))
            print(str(i)+'/'+str(len(test_y))+': Pred: '+str(predictions[-1])+' Real: '+str(y_sample[0])+' in '+str(time.time()-start)+'s')
            pred = int(predictions[-1] >= 0.5)
            ans = int(y_sample[0] >= 0.5)
            if pred == ans: correct += 1
            else: incorrect += 1
            i = i + 1
        return float(100.0*correct)/(correct+incorrect)

    def initialize(self, sess):
        self.sess = tf.get_default_session()
        self.evaluator.initialize(sess)
    def train_learner(self):
        self.train_org_learner()
        # self.evaluator.restore()
        #valid = load_data('./data')
        #print('Valid Accuracy: '+str(self.test(valid)))
        #valid = load_data('./valid')
        #print('Train Accuracy: '+str(self.test(valid)))

    # This needs to be connected to our environment to do anything
    # Need not return anything, just needs to execute the action in gazebo
    def __exec_move__(self, obj_id, x_to, y_to, theta_to):
        #new_state = ModelState()
        #print(x_from)
        #print(y_from)
        #obs_shape = np.reshape(self.obs, [60,60])
        #new_state.model_name = obs_shape[x_from][y_from]
        #new_state.model_name = self.decoder[obj_id]
        #new_state.pose = Pose( Point(x_to,y_to, 0.7), Quaternion(0,0,0,0))
        #new_state.pose.orientaion = rotate_z(new_state.pose, 0, 0, theta_to)
        #self.gazebo_client.gazebo_client.set_pose(new_state)
        new_state = ModelState()
        if self.decoder[obj_id] is not None:
            new_state.model_name = self.decoder[obj_id]
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
        self.decoder = [None]
        for entry in obs_list:
            raw_id = entry[0]
            if raw_id not in names:
                names[raw_id] = len(names)+1
                self.decoder.append(raw_id)
            proc_id = names[raw_id]
            # SOMETHING WRONG HERE
            #print('('+str(entry[1])+','+str(entry[2])+')')
            x = int(round(100*entry[1])-170)
            y = int(round(100*entry[2])+30)
            if x < 60 and x >= 0 and y >= 0 and y < 60:
                obs_array[x][y] = proc_id
        return np.hstack(obs_array)

    def __get_img__(self):
        image = self.image_sub.get_rgb()
        # print(image)
        image = cv2.resize(image, (320, 240))
        return np.array([image])

    def reset(self):
        if self.ep_nb > 0:
            self.last_100_eps.append(self.ep_rew)
            self.last_100_finals.append(self.score)
            print('Episode '+str(self.ep_nb)+' Total Reward: '+str(self.ep_rew)+'\nAvg Total Rew: '+str(float(sum(self.last_100_eps))/len(self.last_100_eps)))
            print('Episode '+str(self.ep_nb)+' Final Score: '+str(self.score)+'\nAvg Final Score: '+str(float(sum(self.last_100_finals))/len(self.last_100_finals))+'\n')

        self.ep_nb += 1
        print('Episode '+str(self.ep_nb)+' Starting...')
        self.__init_episode__()
        self.ep_rew = 0.0
        self.t = 0
        self.r = 0
        self.score = 0.0
        self.obs = self.__get_obs__()
        image = self.__get_img__()
        #print(image[0].shape)
        #if self.t == 0:
        cv2.imwrite('./images/ep_'+str(self.ep_nb)+'_'+str(self.t)+'.png', image[0])
        return self.obs

    def step(self, action):
        #x_from = int(round(30*(action.item(0))))+29
        #y_from = int(round(30*(action.item(1))))+29
        obj_id = int(round(np.max(self.obs)*action.item(0)))
        if obj_id > 0:
            x_to = 0.3*action.item(1)+2.0
            y_to = 0.3*action.item(2)
            theta_to = action.item(3)*math.pi
            #print(action)
            #print('Move From: ('+str(x_from)+','+str(y_from)+') To: '+str(100*x_to)+','+str(100*y_to)+') At '+str(180*theta_to/math.pi))
            self.__exec_move__(obj_id, x_to, y_to, theta_to)
        self.t += 1

        done = (self.t > self.max_t or obj_id == 0)
        self.obs = self.__get_obs__()
        img = self.__get_img__()
        if done:
            cv2.imwrite('./images/ep_'+str(self.ep_nb)+'_end.png', img[0])
        else:
            cv2.imwrite('./images/ep_'+str(self.ep_nb)+'_'+str(self.t)+'.png', img[0])
        new_score = self.evaluator.predict(img)
        diff = new_score-self.score
        self.r = np.sign(diff)*100**(np.abs(diff))
        self.score = new_score
        #print(r)
        self.ep_rew += self.r
        return self.obs, self.r, done, {}
