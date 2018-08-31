import tensorflow as tf
import numpy as np






tf.reset_default_graph()
counter = 2
discount_fact = 0.99

Weights = []
some_layers = []
biases = []

x_one = tf.placeholder(tf.float64, [None, input_dim], name = "x_one")
W1 = tf.get_variable("W1", shape=[input_dim, layers[0]], initializer = tf.contib.layers.xavier_initializer())
b1 = tf.zeros(tf.float64, shape[input_dim,layers[0]])

Weights.append(W1)
biases.append(b1)

layer1 = tf.nn.relu(np.matmul(x_one,W1) + b1)
some_layers.append(layer1)
for i in layers:
    if i == len(layers) - 1:

        Weights.append(tf.get_variable("W" + str(counter), [layers[i], 1], initializer = tf.contib.layers.xavier_initializer()))
        biases.append(tf.zeros(tf.float64, shape[layers[i], 1]))
        prob = .5*(1 - tf.nn.tanh(np.matmul(some_layers[i-1],Weights[i]) + biases[i]))
        
    else:

        Weights.append(tf.get_variable("W" + str(counter), [layers[i], layers[i+1]], initializer = tf.contib.layers.xavier_initializer()))
        biases.append(tf.zeros(tf.float64, shape=[layers[i], layers[i+1]]))
        some_layers.append(tf.nn.relu(np.matmul(some_layers[i],Weights[i]) + biases[i]))

    counter+=1
tvars = tf.trainable_variables()
input_y = tf.placeholder(tf.float64, [None,1])
adv = tf.placeholder(tf.float64, name="reward_sig")

loglik = tf.log(input_y*(input_y - prob) + (1 - input_y)*(input_y + prob))
loss = -tf.reduce_mean(loglik * adv) 
newGrads = tf.gradients(loss,tvars)

nadam = tf.contrib.opt.NadamOptimizer(lr = 0.001, epsilon = 1e-8)

WGrads = []

for i in range(1, len(Weights) + 1):
    WGrads.append(tf.placeholder(tf.float64, name="batch_grad" + str(i)))

updateGrads = nadam.apply_gradients(zip(WGrads, tvars))

def discounted_reward(r):
    discounted_r = np.zeros_like(r)
    running_add = 0
    for t in reversed(xrange(0, r.size)):
        running_add = running_add * discount_fact + r[t]
        discounted_r[t] = running_add
    return discounted_r


drs, xs, ys, hs = []
running_r = None
reward_sum = 0
episode_number = 1
total_episodes = 10000
init = tf.initialize_all_variables()

with tf.Session() as sess:
    


