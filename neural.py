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
              np.random.randn(col,row) * 0.1
          )
          self.bias_vectors.append(
              np.random.randn(col,1)
          )
def linear_activation_forward(A_prev,
                              W,
                              b,
                              activation):
    """
    Implement the forward propagation for the LINEAR->ACTIVATION layer

    Arguments:
    A_prev -- activations from previous layer (or input data): (size of previous layer, number of examples)
    W -- weights matrix: numpy array of shape (size of current layer, size of previous layer)
    b -- bias vector, numpy array of shape (size of the current layer, 1)
    activation -- the activation to be used in this layer, stored as a text string: "sigmoid" or "relu"

    Returns:
    A -- the output of the activation function, also called the post-activation value
    cache -- a python dictionary containing "linear_cache" and "activation_cache";
             stored for computing the backward pass efficiently
    """

    #Define linear forward function, which is really just a way to multiply matrices together
    def linear_forward(activation_vector,  # activations from previous layer
                       weight_matrix,  # weight matrix 
                       bias_vector   # bias matrix
                      ):
    
        Z = np.dot( weight_matrix , activation_vector ) + bias_vector
        # Sanity Check to ensure that the result's shape is actually valid
        assert( Z.shape == (weight_matrix.shape[0], activation_vector.shape[1]) )
        cache = (activation_vector, weight_matrix, bias_vector)
        return Z, cache

    #Apply linear_forward on the activation vector from the previous layer using the given weight matrix and bias vector.
    Z, linear_cache = linear_forward(A_prev, W, b)  

    # Apply the appropiate activation function
    if activation == "sigmoid":
        A, activation_cache = tf.nn.sigmoid(Z)
    elif activation == "relu":
        A, activation_cache = tf.nn.relu(Z)

    # Sanity check to ensure that the new shape is valid
    assert (A.shape == (W.shape[0], A_prev.shape[1]))


    cache = (linear_cache, activation_cache)
    return A, cache
