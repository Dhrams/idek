import numpy as np
import math
from Arduino import Arduino


class Action():
    def __init__():
        self.drive_mag = 0
        self.board = Arduino("115200", port = "/dev/tty.wchusbserial1420")
        self.min_val = -60
        self.max_val = 30
        self.tao = 0.01

    def act(self, action):
        self.drive_mag = 10 if action == 1 else -10
        board.Servos.attach(0)
        C1 =
        C2 =
        thetadd = C1 * trymap(self.drive_mag, 0, 1000, 1000, 2000)**2 + C2 * theta
        theta_dot += thetadd*self.tao
        theta = filter(-.3656*board.analogRead(6) * 185.64)

        
