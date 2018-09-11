import numpy as py
import math
from Arduino import Arduino


class Action():
    def __init__(self):
        self.drive_mag = 0
        self.min_val = -60
        self.max_val = 30
        self.steps_beyond_done = None
        self.state = None
        self.board = Arduino()

    def act(self, action, drive):
        self.drive_mag = 5 if action == 1 else -5
        state = self.state
        [theta_dot, theta] = state
        drive += self.drive_mag



        
