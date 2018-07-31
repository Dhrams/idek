import numpy as py
import math

class Action():
    def __init__(self):
        self.drive_mag=10
        self.min_val = -60
        self.max_val = 30

    def act(self, action):
        self.drive_mag = 10 if action == 1 else -10
        state = 
        [theta_dot, theta] = state

        thetadd = 
