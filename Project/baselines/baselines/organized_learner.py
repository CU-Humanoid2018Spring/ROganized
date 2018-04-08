import tensorflow as tf
import numpy as np

class OrgLearner:
    def __init__(self, in_sess=None, h=240, w=320):
        # Hyperparams
        lr = 1e-4
        drop_rate = 0.5

        # Model initialization
        self.x = tf.placeholder(dtype=tf.float32, shape=([None, h, w, 3]))
        self.y = tf.placeholder(dtype=tf.float32, shape=([None, 1]))
        self.logit = self.__model__(self.x, drop_rate=drop_rate)
        self.out = tf.nn.sigmoid(self.logit)
        self.loss = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(labels=self.y, logits=self.logit))
        # self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(labels=self.y, logits=self.logit))

        #
        # self.loss = -tf.reduce_mean(self.y*tf.log(self.out) + (1-self.y)*tf.log(1-self.out))
        self.train_step = tf.train.AdamOptimizer(lr).minimize(self.loss)

        self.saver = tf.train.Saver()
        if in_sess:
            self.sess = in_sess
        else:
            self.sess = tf.Session()
        self.finalized=False

    # Model building
    def __model__(self, x, drop_rate):
        x = tf.layers.batch_normalization(x)
        # x = tf.layers.average_pooling2d(inputs=x, pool_size=[4,4], strides=4)
        x = tf.layers.conv2d(inputs=x, filters=16, kernel_size=[3, 3], strides=1, padding="same",
                                 activation=tf.nn.relu)
        x = tf.layers.max_pooling2d(inputs=x, pool_size=[2, 2], strides=2)
        x = tf.layers.conv2d(inputs=x, filters=32, kernel_size=[3, 3], strides=1, padding="same",
                                 activation=tf.nn.relu)
        # x = tf.layers.max_pooling2d(inputs=x, pool_size=[2, 2], strides=2)
        #
        # x = tf.layers.conv2d(inputs=x, filters=16, kernel_size=[1, 1], strides=1, padding="same",
        #                          activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=128, kernel_size=[3, 3], strides=1, padding="same",
        #                          activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=16, kernel_size=[1, 1], strides=1, padding="same",
        #                          activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=128, kernel_size=[3, 3], strides=1, padding="same",
        #                          activation=tf.nn.relu)
        # x = tf.layers.max_pooling2d(inputs=x, pool_size=[2, 2], strides=2)
        # x = tf.layers.conv2d(inputs=x, filters=32, kernel_size=[1, 1], strides=1, padding="same",
        #                          activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=256, kernel_size=[3, 3], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=32, kernel_size=[1, 1], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=256, kernel_size=[3, 3], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.max_pooling2d(inputs=x, pool_size=[2, 2], strides=2)
        # x = tf.layers.conv2d(inputs=x, filters=64, kernel_size=[1, 1], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=512, kernel_size=[3, 3], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=64, kernel_size=[1, 1], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=512, kernel_size=[3, 3], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=128, kernel_size=[1, 1], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.conv2d(inputs=x, filters=1000, kernel_size=[1, 1], strides=1, padding="same",
        #                           activation=tf.nn.relu)
        # x = tf.layers.average_pooling2d(inputs=x, pool_size=[x.shape[1], x.shape[2]], strides=1)
        x = tf.layers.flatten(x)

        # x = tf.layers.dense(x, 512)
        # x = tf.layers.dropout(x, rate=drop_rate)
        # x = tf.nn.relu(x)
        # x = tf.layers.dense(x, 256)
        # x = tf.layers.dropout(x, rate=drop_rate)
        # x = tf.nn.relu(x)
        x = tf.layers.dense(x, 128)
        x = tf.layers.dropout(x, rate=drop_rate)
        x = tf.nn.relu(x)
        x = tf.layers.dense(x, 1)
        # x = tf.nn.sigmoid(x)
        return x

    # Shuffles and batches data, returns a list of np arrays of batch_size
    def __get_next_batch__(self, data, batch_size, i, p):
        x_batches = []
        y_batches = []

        # Shuffling X, Y
        Y = np.array(data[1])
        Y = np.transpose(np.expand_dims(Y, axis=0))
        if batch_size <= 0:
            batch_size = len(data[0])

        # Batching
        while i < len(data[0]):
            x_tmp = [data[0][i] for i in p[i:i + batch_size]]
            x_batches.append(np.array(x_tmp))
            y_batches.append(Y[p[i:i+batch_size]])
            # i = i + batch_size + 1
        return [x_batches, y_batches]

    # Data has to be in the form of [X,Y], where X is a list of np arrays, and y is a list of single values [0-1]
    def train(self, data, batch_size=32):
        if not self.finalized:
            self.sess.run(tf.global_variables_initializer())
            self.sess.graph.finalize()
            self.finalized=True

        p = np.random.permutation(len(data[1]))
        i = 0
        # X, Y = self.__get_next_batch__(data, batch_size)
        Y = np.array(data[1])[p]
        Y = np.transpose(np.expand_dims(Y, axis=0))
        loss = 0.0
        while i < len(data[0]):
            x_tmp = [data[0][j] for j in p[i:i + batch_size]]
            y_tmp = [data[1][j] for j in p[i:i + batch_size]]
            x_batch = np.array(x_tmp)
            y_batch = np.array(y_tmp)
            if len(y_batch.shape) < 2:
                y_batch = np.transpose(np.expand_dims(y_batch, axis=0))
            outs, L, _ = self.sess.run([self.out, self.loss, self.train_step], feed_dict={self.x: x_batch, self.y: y_batch})
            # print(L)
            i = i + batch_size
            import math
            if math.isnan(loss):
                print(i)
                print(outs)
            loss += L
        return batch_size*loss/i

    # Output [0-1] for all predicted values with input x
    def predict(self, x):
        if not self.finalized:
            self.sess.run(tf.global_variables_initializer())
            self.sess.graph.finalize()
            self.finalized=True
        pred = self.sess.run([self.out], feed_dict={self.x: x})
        return pred[0][0][0]

    # Save the model
    def save(self, path='./organized_model.ckpt'):
        self.saver.save(self.sess, path)

    # Restores the model
    def restore(self, path='./organized_model.ckpt'):
        self.saver.restore(self.sess, path)
        self.finalized=True

def get_files_in_dir(path):
    from os import listdir
    from os.path import isfile, join
    files = [join(path, f) for f in listdir(path) if isfile(join(path, f))]
    dirs = [join(path, f) for f in listdir(path) if not isfile(join(path, f))]
    for dir in dirs:
        rec_files = get_files_in_dir(dir)
        files.extend(rec_files)
    return files

def load_data(path):
    import random
    import cv2
    X = []
    Y = []
    messy = get_files_in_dir(path+'/messy')
    for file in messy:
        x = cv2.imread(file)
        x = cv2.resize(x, (320, 240))
        # x[0][0] = 1
        X.append(x)
        # Y.append(np.array([0,1]))
        Y.append(max(0.0, random.normalvariate(0.1, 0.2)))
    nb_messy = len(Y)
    print('Loaded '+str(len(Y))+' messy images')
    neat = get_files_in_dir(path+'/neat')
    for file in neat:
        x = cv2.imread(file)
        # cv2.imshow('img', x)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        x = cv2.resize(x, (320, 240))
        # cv2.imshow('img', x)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # x[0][0]=0
        X.append(x)
        # Y.append(np.array([1,0]))
        Y.append(min(1.0, random.normalvariate(0.9, 0.2)))
    print('Loaded '+str(len(Y)-nb_messy)+' neat images')
    print('Loaded '+str(len(Y))+' total images')
    return X, Y

# Runs a full suite of unit tests (initing, training, saving, restoring, and testing) and saves the model
if __name__ == '__main__':
    import time
    data = load_data('/home/robert/Documents/ROganized/Project/baselines/baselines/data')
    # test_data = load_data('/home/robert/Documents/ROganized/Project/baselines/baselines/test')
    test_data = data
    test_x = test_data[0]
    test_y = test_data[1]
    p = np.random.permutation(len(test_y))
    epochs = 10
    ol = OrgLearner()
    print('Training Started')
    for i in range(epochs):
        start = time.time()
        loss = ol.train(data)
        print('Epoch '+str(i)+'/'+str(epochs)+': Loss: '+str(loss)+' Completed In: '+str(time.time()-start)+'s')
    print('Training Complete')
    ol.save()
    print('Save Complete')
    del ol
    tf.reset_default_graph()
    ol = OrgLearner()
    ol.restore()
    print('Restore Complete')
    i = 0
    correct = 0
    incorrect = 0
    predictions = []
    test_y = np.array(test_y)
    Y = np.transpose(np.expand_dims(test_y, axis=0))
    print('Testing Started')
    while i < len(test_x):
        x_sample = [test_x[j] for j in p[i:i + 1]]
        y_sample = [test_y[j] for j in p[i:i + 1]]
        x_batch = np.array(x_sample)
        y_batch = np.array(y_sample)
        start = time.time()
        predictions.append(ol.predict(x_sample))
        # print(str(i)+'/'+str(len(data[0]))+': Pred: '+str(predictions[-1])+' Real: '+str(y_sample[0])+' in '+str(time.time()-start)+'s')
        pred = int(predictions[-1] >= 0.5)
        ans = int(y_sample[0] >= 0.5)
        if pred == ans: correct += 1
        else: incorrect += 1
        i = i + 1
    print('Testing Complete')
    print(str(correct)+' correct')
    print('Final Accuracy: '+str(100*float(correct)/float(correct+incorrect))+'%')
