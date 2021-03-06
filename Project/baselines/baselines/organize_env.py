import gym
from gym import spaces
import numpy as np
from collections import deque
import tensorflow as tf
import math
import time
from baselines.organized_learner import OrgLearner, load_data
from baselines.scene_generator import random_poses
from table_manager import TableManager
from utils import ImageConverter
import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from geometry_msgs.msg import Quaternion, Pose, Twist, Point
import cv2
from moveit_python.geometry import rotate_pose_msg_by_euler_angles as rotate

class OrganizeEnv(gym.Env):
    def __init__(self):
        self.h = 5
        self.w = 5
        self.max_t = 50
        self.last_1000_eps = deque(maxlen=1000)
        self.last_1000_finals = deque(maxlen=1000)
        self.ep_rew = 0.0
        self.ep_nb = 0
        self.r = 0
        self.action_space = spaces.Box(-1, 1, shape=(3,))
        self.observation_space = spaces.Box(low=np.zeros(shape=[self.w*self.h,]), high=256*np.ones(shape=[self.w*self.h,]))
        self.evaluator = OrgLearner()
        self.log_file = open(str(int(time.time()))+'_log.txt', 'w+')


        rospy.init_node("rl_env")
        self.table = TableManager()
        rospy.sleep(0.1)
        self.table.clear()
        self.table.spawn()
        self.image_sub = ImageConverter()
        self.reset()
 
    def train_org_learner(self):
        data = load_data('../data/scored_with_poses_30k')
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
        #self.evaluator.restore()
        #valid = load_data('./data')
        #print('Valid Accuracy: '+str(self.test(valid)))
        #valid = load_data('./valid')
        #print('Train Accuracy: '+str(self.test(valid)))

    # This needs to be connected to our environment to do anything
    # Need not return anything, just needs to execute the action in gazebo
    def __exec_move__(self, obj_id, x_to, y_to):
        #new_state = ModelState()
        #print(x_from)
        #print(y_from)
        #obs_shape = np.reshape(self.obs, [60,60])
        #new_state.model_name = obs_shape[x_from][y_from]
        #new_state.model_name = self.decoder[obj_id]
        #new_state.pose = Pose( Point(x_to,y_to, 0.7), Quaternion(0,0,0,0))
        #new_state.pose.orientaion = rotate_z(new_state.pose, 0, 0, theta_to)
        #new_state = ModelState()
        # if self.decoder[obj_id] is not None:
            #new_state.model_name = self.decoder[obj_id]
            #new_state.pose = Pose( Point(x_to,y_to, 0.7),\
            #                       Quaternion(0,0,0,1))
            #new_state.pose.orientation = rotate(new_state.pose, 0, 0, theta_to)
        self.table.move_cube(obj_id, x_to, y_to)


    # This needs to be connected to our environment to do anything
    # Need not return anything, juse needs to randomly initialize the gazebo env
    def __init_episode__(self):
        grids = 5
        positions = np.random.choice(grids*grids-1,4,replace=False)
        for i, position in enumerate(positions):
            self.table.move_cube(i, position/grids, position%grids)
        rospy.sleep(0.1)

    # This needs to be connected to our environment to do anything
    # Should return a numpy array of dimensions [self.w, self.h, 3]
    def __get_obs__(self):
        obs_list = []
        obs_array = np.zeros([5,5], dtype=np.uint8)
        for i in range(4):
            p = self.table.models['cube_{}'.format(i)].position
            q = self.table.models['cube_{}'.format(i)].orientation
            obs_list.append([i, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            x = int(round((p.x-0.35)/0.1))
            y = int(round((p.y+0.2)/0.1))
            try:
                obs_array[x][y] = i
            except: pass
        # return np.array(obs_list)
        # names = dict()
        # self.decoder = [None]
        # for entry in obs_list:
        #     raw_id = entry[0]
        #     if raw_id not in names:
        #         names[raw_id] = len(names)+1
        #         self.decoder.append(raw_id)
        #     proc_id = names[raw_id]
        #     # SOMETHING WRONG HERE
        #     #print('('+str(entry[1])+','+str(entry[2])+')')
        #     x = int(round(100*entry[1])-170)
        #     y = int(round(100*entry[2])+30)
        #     if x < 60 and x >= 0 and y >= 0 and y < 60:
        #         obs_array[x][y] = proc_id
        return np.hstack(obs_array)

    def __get_img__(self):
        image = self.image_sub.get_rgb()
        while image is None:
            rospy.logwarn('rgb get None, retrying...')
            rospy.sleep(0.01)
            image = self.image_sub.get_rgb()
        # print(image.shape)
        image = cv2.resize(image, (320, 240))
        return np.array([image])

    def reset(self):
        if self.ep_nb > 0:
            self.last_1000_eps.append(self.ep_rew)
            self.last_1000_finals.append(self.score)
            print('Episode '+str(self.ep_nb)+' Total Reward: '+str(self.ep_rew)+'\nAvg Total Rew: '+str(float(sum(self.last_1000_eps))/len(self.last_1000_eps)))
            print('Episode '+str(self.ep_nb)+' Final Score: '+str(self.score)+'\nAvg Final Score: '+str(float(sum(self.last_1000_finals))/len(self.last_1000_finals))+'\n')
            self.log_file.write(str(self.ep_nb)+' Total: '+str(self.ep_rew)+'\n')
            self.log_file.write(str(self.ep_nb)+' Final: '+str(self.score)+'\n')

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
        obj_id = int(round(np.max(self.obs)*abs(action.item(0))))
        # print(action.item(0))
        if obj_id > 0:
            x_to = int(round(2.0*action.item(1)))+2
            y_to = int(round(2.0*action.item(2)))+2
            # theta_to = action.item(3)*math.pi
            # print(action)
            # print('Move '+str(obj_id)+' To: '+str(x_to)+','+str(y_to)+')')
            self.__exec_move__(obj_id, x_to, y_to)
        self.t += 1

        done = (self.t > self.max_t)
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
