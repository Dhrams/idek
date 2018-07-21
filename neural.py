from keras.layers import LSTM

class NeuralNet(object):

    def __init__(self,
                 input_node_size = None,               # Number of nodes in input layer
                 output_node_size = None,              # Number of nodes in output layer
                 hidden_layers_node_size = []          # Number of nodes in each hidden layer
                ):
                    from keras.models import Sequential
                    self.model = Sequential()
                    from keras.layers import Dense
                    # First layer requires input dimension ie input_node_size

                    self.model.add(LSTM(
                                        units = 64
                                        activation = 'relu'
                                        input_dim = input_node_size
                                        ))

                    # self.model.add(
                    #                Dense(units=64,
                    #                      activation='relu',
                    #                      input_dim=input_node_size
                    #                      )
                    #               )
                    # Add layers to model for all hidden layers
                    for node_size in hidden_layer_node_size:
                        self.model.add(
                                       Dense(units=node_size,
                                             activation='relu'
                                             )
                                      )
                    # Last layer requires activation to be softmax
                    self.model.add(
                                   Dense(units=output_node_size,
                                         activation='softmax'
                                         )
                                  )
                    # Compile model
                    self.model.compile(loss='categorical_crossentropy',
                                       optimizer='sgd',
                                       metrics=['accuracy'])
                    #model.fit(x_train, y_train, epochs=5, batch_size=32)


    """
    fit the model with training datasets

    inputs:
    train_x - training data
    train_y - training labels
    epochs - number of iterations over the entirity of both the x and y data desired

    returns:
    Nothing

    """
    def train(self, train_x, train_y, epochs):
        self.model.fit(train_x, train_y, epochs, batch_size = 32)

    """
    evaluates the model with test data

    inputs:
    X - test data
    Y - test labels
    steps - number of iterations over the entire dataset before evaluation is completed

    returns:
    metrics - the test losses as well as the metric defined in __init__, which in this case is accuracy
    """

    def run(self, X, Y, steps):
        metrics = []
        metrics = self.model.evaluate(X, Y, batch_size = 32, steps = steps)
        return metrics

    """
    predicts the labels of the data given

    Inputs:
    X - unlabeled test data
    steps - number of iterations over the entire dataset before evaluation is completed

    returns:
    predictions - a numpy array of predictions
    """
    def label(self, X, steps):
        predictions = self.model.predict(X, batch_size = 32, steps = steps)
        return predictions
