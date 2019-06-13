import sklearn
from sklearn.cluster import kMeans
import numpy as np
import sklearn.dataset as data
import h5py
import tensorflow as tf

# -------------------------------- KMeans --------------------------------------

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Store in h5 file !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
seed = np.random.seed(3)

kmeans = kMeans(n_clusters = 3, algorithm = "auto", random_state = seed).fit(X)

Y = kmeans.labels_
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Store in h5 file !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# ------------------------------ Neural Net ------------------------------------

# data \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
trainX, trainY, testX, testY, classes = load_data()

# Hyperparameters \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

parameters = {}     # empty dict for weights and biases
LR = 5e-3           # learning rate
iterations = 5000
input_dim = 2
layer_dims = [input_dim, 7, 6, 8, 1] # currently random numbers -> list of sizes of each layer


# !!!!!!!!!!!!!!!!!!!!!!!!!!! Need to flatten data !!!!!!!!!!!!!!!!!!!!!!!!!!!!!


caches = []

L = len(layer_dims) # amount of W + b pairs needed

def nn_model(X, Y, layer_dims, lr = 7.5e-3, iters):
    np.random.seed(1)
    costs = []                         # keep track of cost

    # Parameters initialization.
    parameters = initialize_parameters_deep(layers_dims)

    # Loop (gradient descent)
    for i in range(0, num_iterations):

        # Forward propagation: [LINEAR -> RELU]*(L-1) -> LINEAR -> SIGMOID.
        AL, caches = L_model_forward(X, parameters)

        # Compute cost.
        cost = compute_cost(AL, Y)

        # Backward propagation.
        grads = L_model_backward(AL, Y, caches)

        # Update parameters.
        parameters = update_parameters(parameters, grads, lr)

        # Print the cost every 100 training examples
        if print_cost and i % 100 == 0:
            print ("Cost after iteration %i: %f" %(i, cost))
        if print_cost and i % 100 == 0:
            costs.append(cost)

    return parameters
# forward propagation of L - layer model \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\



# predict ------------------------------------------------

new_params = nn_model(trainX, trainY, layer_dims, lr = 7.5e-3, iterations)

predY = kmeans.predict(something)

prediction = predict(something, predY, new_params)
    #blah -----------------------------------------------
