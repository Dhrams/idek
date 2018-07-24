class NeuralNet(object):

    def __init__(self,
                 input_node_size = None,               # Number of nodes in input layer
                 output_node_size = None,              # Number of nodes in output layer
                 input_shape = None,
                 hidden_layers_node_size = []          # Number of nodes in each hidden layer
                ):
                    from keras.models import Sequential
                    self.model = Sequential()
                    from keras.layers import Dense, Dropout, Activation, Flatten
                    # First layer requires input dimension ie input_shape
                    self.model.add(
                                   LSTM(units=64,
                                         input_dim=input_node_size
                                         )
                                   )
                    self.model.add(Activation('relu'))
                    # Add layers to model for all hidden layers
                    for node_size in hidden_layers_node_size:
                        self.model.add(
                                       Dense(units=node_size)
                                      )
                        self.model.add(Activation('relu'))
                        self.model.add(Dropout(0.3))
                    #          from keras import regularizers
                    #          self.model.add(Dense(64,
                    #                          input_dim=64,
                    #                          kernel_regularizer=regularizers.l2(0.01),
                    #                          activity_regularizer=regularizers.l1(0.01))
                    #                   )
                    # Last layer requires activation to be softmax
                    self.model.add(
                                   Dense(units=output_node_size,
                                         activation='softmax'
                                         )
                                  )
                    # Compile model
                    self.model.compile(loss='categorical_crossentropy',
                                       optimizer='adam',
                                       metrics=['accuracy'])
                    #model.fit(x_train, y_train, epochs=5, batch_size=32)
    def train(self, train_x, train_y, epochs):
        self.model.fit(train_x, train_y, epochs, batch_size = 32)
    def run(self, X, Y, steps):
        metrics = []
        metrics = self.model.evaluate(X, Y, batch_size = 32, steps = steps)
        return metrics
    def train(self, train_x, train_y, epochs):
        self.model.fit(train_x, train_y, epochs, batch_size = 32)
