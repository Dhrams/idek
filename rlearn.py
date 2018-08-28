import tensorflow as tf
import numpy as np


tf.reset_default_graph()

Weights = []
some_layers = []

x_one = tf.placeholder(tf.float64, [None, input_dim], name = "x_one")
W1 = tf.get_variable("W1", shape=[input_dim, layers[0]], initializer = tf.contib.layers.xavier_initializer())
b1 = tf.zeros(tf.float64, shape[input_dim,layers[0]])
layer1 = tf.nn.relu(np.matmul(x_one,W1) + b1)
for i in layers:
    Weights.append(tf.get)
