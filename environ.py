import numpy as np

import roslib
import rospy
import ros_arduino_python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_arduino_msgs.msg import SensorState, ServoRead, ServoWrite
from sensor_msgs.msg import Range


class Action():
    def __init__(self):
        # publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 2)
        # subscribers
        self.
        self.drive_mag = 10
        self.upper_bound = 30
        self.lower_bound = -65
        self.drive = 0

    def step(self, action):
        state = get_observe()
        pitch = filter #....
        done = if pitch > self.upper_bound | pitch < self.lower_bound
        states = get_observe()
        reward = -(abs(states[0] - states[1]))^2
        return [states, reward, done, []]
