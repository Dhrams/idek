import numpy as np
#import sklearn.dataset as data
import h5py
import tensorflow as tf
import sklearn

class NeuralNet(object):
  def __init__(self,
               input_node_size = None,               # Number of nodes in input layer
               output_node_size = None,              # Number of nodes in output layer
               hidden_layers_node_size = []          # Number of nodes in each hidden layer
              ):
  
      # self.weight_matrices
      # self.bias_vectors
  
      """
  
      init function for neuralNet
  
      Takes in the size of node layers (where each node layer is a number of nodes)
      Creates weight matrices and bias vectors with random values
      Function returns nothing.
  
      """
  
      # Randomize function seed
      np.random.seed(3)
  
      # Input checks
      if input_node_size is None or output_node_size is None:
          raise ValueError("input_node_size and output_node _size cannot be None")
      elif input_node_size < 1 or output_node_size < 1:
          raise ValueError("input_node_size and output_node_size must be greater than 0")
  
      # Creation of dimensions for weight matrices
      cols_size = [input_node_size] + hidden_layers_node_size
      rows_size = hidden_layers_node_size + [output_node_size]
      dimensions = zip(cols_size,rows_size)
  
      # Storage of weight and bias matrices
      self.weight_matrices = []
      self.bias_vectors = []
  
      for i, (col,row) in enumerate(dimensions):
          self.weight_matrices.append(
              np.random.randn(row,col) * 0.1
          )
          self.bias_vectors.append(
              np.random.randn(row,1)
          )
  def L_model_forward(self,input_vector):
      """
      Implement forward propagation for the [LINEAR->RELU]*(L-1)->LINEAR->SIGMOID computation
  
      Arguments:
      X -- data, numpy array of shape (input size, number of examples)
  
      Returns:
      AL -- last post-activation value
      caches -- list of caches containing:
                  every cache of linear_activation_forward() (there are L-1 of them, indexed from 0 to L-1)
      """
  
      def forward(activation_vector,
                  weight_matrix,
                  bias_vector,
                  activation):
          Z = np.dot( weight_matrix , activation_vector ) + bias_vector
          assert( Z.shape == (weight_matrix.shape[0], activation_vector.shape[1]) )
          linear_cache = (activation_vector, weight_matrix, bias_vector)
          # Apply the appropiate activation function
          if activation == "sigmoid":
              A = 1/(1+np.exp(-Z))
      
          elif activation == "relu":
              A = Z*(Z > 0)
          # Sanity check to ensure that the new shape is valid
          assert (A.shape == (weight_matrix.shape[0], activation_vector.shape[1]))
          cache = linear_cache
          return A, cache
  
      # Store caches from each operation
      caches = []
      activation_vector = input_vector
  
      # Implement LINEAR -> RELU for all matrices except last one. 
      # Also add "cache" to the "caches" list.
      for weight_matrix,bias_vector in zip(self.weight_matrices[:-1], self.bias_vectors[:-1]):
          A_prev=activation_vector
          activation_vector,cache=forward(A_prev,
                                          weight_matrix, 
                                          bias_vector, 
                                          activation = "relu")
          # Store cache of operation in caches
          caches.append(cache)
  
      # Implement LINEAR -> SIGMOID for last matrix. Add "cache" to the "caches" list.
      AL,cache=forward(activation_vector,
                       self.weight_matrices[-1],
                       self.bias_vectors[-1],
                       activation = "sigmoid")
      caches.append(cache)
  
      return AL, caches
  def compute_cost(self,AL, Y):
      """
      Implement the cost function defined by equation (7).
  
      Arguments:
      AL -- probability vector corresponding to your label predictions, shape (1, number of examples)
      Y -- true "label" vector (for example: containing 0 if non-cat, 1 if cat), shape (1, number of examples)
  
      Returns:
      cost -- cross-entropy cost
      """
  
      m = Y.shape[1]
  
      # Compute loss from aL and y.
      cost = -1/m*np.sum(np.multiply(Y, np.log(AL)) + np.multiply((1-Y), np.log(1-AL)))
  
      cost = np.squeeze(cost)      # To make sure your cost's shape is what we expect (e.g. this turns [[17]] into 17).
      assert(cost.shape == ())
  
      return cost
  def L_model_backward(self,AL, Y, caches):
      """
      Implement the backward propagation for the [LINEAR->RELU] * (L-1) -> LINEAR -> SIGMOID group
  
      Arguments:
      AL -- probability vector, output of the forward propagation (L_model_forward())
      Y -- true "label" vector (containing 0 if non-cat, 1 if cat)
      caches -- list of caches containing:
                  every cache of linear_activation_forward() with "relu" (it's caches[l], for l in range(L-1) i.e l = 0...L-2)
                  the cache of linear_activation_forward() with "sigmoid" (it's caches[L-1])
  
      Returns:
      grads -- A dictionary with the gradients
               grads["dA" + str(l)] = ...
               grads["dW" + str(l)] = ...
               grads["db" + str(l)] = ...
      """
  
      def linear_activation_backward(dA, cache, activation):
          """
          Implement the backward propagation for the LINEAR->ACTIVATION layer.
      
          Arguments:
          dA -- post-activation gradient for current layer l
          cache -- tuple of values (linear_cache, activation_cache) we store for computing backward propagation efficiently
          activation -- the activation to be used in this layer, stored as a text string: "sigmoid" or "relu"
      
          Returns:
          dA_prev -- Gradient of the cost with respect to the activation (of the previous layer l-1), same shape as A_prev
          dW -- Gradient of the cost with respect to W (current layer l), same shape as W
          db -- Gradient of the cost with respect to b (current layer l), same shape as b
          """
          def linear_backward(dZ, cache):
              """
              Implement the linear portion of backward propagation for a single layer (layer l)
          
              Arguments:
              dZ -- Gradient of the cost with respect to the linear output (of current layer l)
              cache -- tuple of values (A_prev, W, b) coming from the forward propagation in the current layer
          
              Returns:
              dA_prev -- Gradient of the cost with respect to the activation (of the previous layer l-1), same shape as A_prev
              dW -- Gradient of the cost with respect to W (current layer l), same shape as W
              db -- Gradient of the cost with respect to b (current layer l), same shape as b
              """
              A_prev, W, b = cache
              m = A_prev.shape[1]
          
              dW = 1/m * np.dot(dZ, A_prev.T)
              db = 1/m*np.sum(dZ, axis = 1, keepdims = True)
              dA_prev = np.dot(W.T, dZ)
          
              assert (dA_prev.shape == A_prev.shape)
              assert (dW.shape == W.shape)
              assert (db.shape == b.shape)
          
              return dA_prev, dW, db
      
          linear_cache, activation_cache = cache
      
          if activation == "relu":
              dZ = relu_backward(dA, activation_cache)
              dA_prev, dW, db = linear_backward(dZ, linear_cache)
      
          elif activation == "sigmoid":
              dZ = sigmoid_backward(dA, activation_cache)
              dA_prev, dW, db = linear_backward(dZ, linear_cache)
      
          return dA_prev, dW, db
  
      grads = {}
      L = len(caches) # the number of layers
      m = AL.shape[1]
      Y = Y.reshape(AL.shape) # after this line, Y is the same shape as AL
  
      # Initializing the backpropagation
      dAL = - (np.divide(Y, AL) - np.divide(1 - Y, 1 - AL))
  
      # Lth layer (SIGMOID -> LINEAR) gradients. Inputs: "dAL, current_cache". Outputs: "grads["dAL-1"], grads["dWL"], grads["dbL"]
      grads["dA" + str(L-1)], grads["dW" + str(L)], grads["db" + str(L)] = linear_activation_backward(dAL, current_cache, activation = "sigmoid")
  
      # Loop from l=L-2 to l=0
      for l in reversed(range(L-1)):
          # lth layer: (RELU -> LINEAR) gradients.
          # Inputs: "grads["dA" + str(l + 1)], current_cache". Outputs: "grads["dA" + str(l)] , grads["dW" + str(l + 1)] , grads["db" + str(l + 1)]
          current_cache = caches[l]
          dA_prev_temp, dW_temp, db_temp = linear_activation_backward(grads["dA" + str(L-1)], caches[l], activation = "relu")
          grads["dA" + str(l)] = dA_prev_temp
          grads["dW" + str(l + 1)] = dW_temp
          grads["db" + str(l + 1)] = db_temp
  
      return grads

if __name__ is "__main__":
    a = NeuralNet(3,4,[3,4])
    print("Hello")
    print(a.weight_matrices)
    a.L_model_forward(np.array([[3],[4],[3]]))
