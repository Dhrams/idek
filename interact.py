import numpy as np
import roslib
import rospy
import ros_arduino_msgs


class test():
    def __init__(self):
        self.servo_writemicroseconds = rospy.ServiceProxy('ServoWritemicroseconds', ros_arduino_msgs.srv.ServoWritemicroseconds)
        self.drive = 200

    def trymap(x, in_min, in_max, out_min, out_max):
        return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

    def trymicro(self):
        us = trymap(self.drive, 0, 1000, 1000, 2000)
        self.servo_writemicroseconds(6, us)

if __name__ == "__main__":
    trymicro()
