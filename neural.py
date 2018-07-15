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
                    self.model.add(
                                   Dense(units=64,
                                         activation='relu',
                                         input_dim=input_node_size
                                         )
                                  )
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
