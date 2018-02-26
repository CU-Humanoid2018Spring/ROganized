import tensorflow as tf
import numpy as np

class OrgLearner:
    def __init__(self, in_sess=None, h=144, w=256):
        self.saver = tf.train.Saver()
        if in_sess:
            self.sess = in_sess
        else:
            self.sess = tf.Session()
            self.sess.graph.finalize()

        # Hyperparams
        lr = 1e-4
        drop_rate = 0.5

        # Model initialization
        self.x = tf.placeholder(dtype=tf.float32, shape=([None, w, h, 3]))
        self.y = tf.placeholder(dtype=tf.float32, shape=([None, 1]))
        self.out = self.__model__(self.x, drop_rate=drop_rate)
        self.loss = tf.nn.softmax_cross_entropy_with_logits_v2(labels=self.y, logits=self.out)
        self.train_step = tf.train.AdamOptimizer(lr).minimize(self.loss)

    # Model building
    def __model__(self, x, drop_rate):
        x = tf.layers.batch_normalization(x)
        x = tf.layers.conv1d(x, 64, 5)
        x = tf.layers.conv1d(x, 32, 3)
        x = tf.layers.dense(x, 512)
        x = tf.layers.dropout(x, rate=drop_rate)
        x = tf.nn.leaky_relu(x)
        x = tf.layers.dense(x, 256)
        x = tf.layers.dropout(x, rate=drop_rate)
        x = tf.nn.leaky_relu(x)
        x = tf.layers.dense(x, 128)
        x = tf.layers.dropout(x, rate=drop_rate)
        x = tf.nn.leaky_relu(x)
        x = tf.layers.dense(x, 1)
        x = tf.nn.sigmoid(x)
        return x

    # Shuffles and batches data, returns a list of np arrays of batch_size
    def __get_batches__(self, data, batch_size):
        if batch_size <= 0:
            return data
        x_batches = []
        y_batches = []
        # Shuffling X, Y
        p = np.random.permutation(len(data[1]))
        X = np.array(data[0])[p]
        Y = np.array(data[1])[p]

        # Batching them
        while len(data) >= batch_size:
            x_batches.append(X[:batch_size])
            y_batches.append(Y[:batch_size])
            X = X[batch_size:]
            Y = Y[batch_size:]
        return [x_batches, y_batches]

    # Data has to be in the form of [X,Y], where X is a list of np arrays, and y is a list of single values [0-1]
    def train(self, data, batch_size=-1):
        X, Y = self.__get_batches__(data, batch_size)
        loss = 0.0
        for i in range(len(Y)):
            L, _ = self.sess.run([self.loss, self.train_step], feed_dict={self.x: X[i], self.y: Y[i]})
            loss += L
        return loss

    # Output [0-1] for all predicted values with input x
    def predict(self, x):
        return self.sess.run([self.out], feed_dict={self.x: x})

    # Save the model
    def save(self, path='organized_model.ckpt'):
        self.saver.save(self.sess, path)

    # Restores the model
    def restore(self, path='organized_model.ckpt'):
        self.saver.restore(self.sess, path)

# Runs a full suite of unit tests (initing, training, saving, restoring, and testing) and saves the model
if __name__ == '__main__':
    data = []
    test_x = []
    test_y = []
    epochs = 100
    ol = OrgLearner()
    for i in range(epochs):
        loss = ol.train(data)
        print('Train Step: '+str(i)+' Loss: '+str(loss))
    ol.save()
    print('Training Complete')
    del ol
    ol = OrgLearner()
    ol.restore()
    predictions = ol.predict(test_x)

    correct = 0
    incorrect = 0
    for i in range(len(test_y)):
        print('Pred: '+str(predictions[i])+' Real: '+str(test_y[i]))
        logit = int(predictions[i] >= 0.5)
        if logit == test_y[i]: correct += 1
        else: incorrect += 1
    print('Testing Complete')
    print('Final Accuracy: '+str(float(correct)/float(correct+incorrect))+'%')
