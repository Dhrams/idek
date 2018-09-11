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
        prob = 500*(1 - tf.nn.tanh(np.matmul(some_layers[i-1],Weights[i]) + biases[i]))
        
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

nadam = tf.contrib.opt.NadamOptimizer(lr = 0.01, epsilon = 1e-8)

WGrads = []
BGrads = []

for i in range(1, len(Weights) + 1):
    WGrads.append(tf.placeholder(tf.float64, name="batch_grad" + str(i)))

updateGrads = nadam.apply_gradients(zip(WGrads, tvars))

def get_observe(self):
    '''
    need to put csv parser here
    '''

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
episode = True
episode_number = 1
init = tf.global_variables_initializer()

with tf.Session() as sess:
    sess.run(init)
    observation = get_observe()

    gradBuffer = sess.run(tvars)
    for ix,grad in enumerate(gradBuffer):
        gradBuffer[ix] = grad * 0
    while episode:

        x = np.reshape(observation,[1,input_dim])
        tfprob = sess.run(prob, feed_dict = {observations: x})
        action = 0 if tfprob < 200 else 1

        xs.append(x)
        y = 1 if action == 0 else 0 # need to find a way to get actual labels
        ys.append(y)

        observation, reward, done, info = step(action) # make class for this stuff

        reward_sum += reward
        drs.append(reward)

        if done:
            episode_number += 1
            epx = np.vstack(xs)
            epy = np.vstack(ys)
            epr = np.vstack(drs)
            xs,hs,drs,ys = [],[],[],[]

             # compute the discounted reward backwards through time
            discounted_epr = discount_rewards(epr)
            # size the rewards to be unit normal (helps control the gradient estimator variance)
            discounted_epr -= np.mean(discounted_epr)
            discounted_epr /= np.std(discounted_epr)

            tGrad = sess.run(newGrads,feed_dict={x_one: epx, input_y: epy, adv: discounted_epr})
            for ix,grad in enumerate(tGrad):
                gradBuffer[ix] += grad
                
            # If we have completed enough episodes, then update the policy network with our gradients.
            if episode_number % batch_size == 0: 
                sess.run(updateGrads,feed_dict={WGrad: gradBuffer})
                for ix,grad in enumerate(gradBuffer):
                    gradBuffer[ix] = grad * 0
                
                # Give a summary of how well our network is doing for each batch of episodes.
                running_r = reward_sum if running_r is None else running_r * 0.99 + reward_sum * 0.01
                print 'Average reward for episode %f.  Total average reward %f.' % (reward_sum/batch_size, running_r/batch_size)

                if reward_sum/batch_size > 300:
                    print "Model is now fully trained... would you like to input an angle?"
                    
                reward_sum = 0
            
            observation = get_observe()
        
print episode_number,'Episodes completed.'
