import numpy as np

import roslib
import rospy
import ros_arduino_python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import ros_arduino_msgs
import ros_arduino_msgs.srv as rsrv


class Action():
    def __init__(self):
        # publishers
        
        # subscribers
        self.servo_wMs = rospy.ServiceProxy('servo_writemicroseconds',rsrv.ServoWritemicroseconds)
        self.drive_mag = 10
        self.upper_bound = 30
        self.lower_bound = -65
        self.drive = 0

    def trymap(self, x, in_min, in_max, out_min, out_max):
        return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

    def step(self, action):
        
        state = rlearn.get_observe()
        done = if state[0] > self.upper_bound | state[0] < self.lower_bound
        servo_wMs(6,action)
        states = get_observe()
        reward = -(abs(states[0] - states[1]))^2
        return [states, reward, done, []]
 
