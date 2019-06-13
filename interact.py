import numpy as np
import roslib
import rospy
import ros_arduino_msgs
import ros_arduino_msgs.srv as rsrv


class test():
    def __init__(self):
        rospy.wait_for_service('ServoRead')
        self.digital_write = rospy.ServiceProxy('DigitalWrite', rsrv.DigitalWrite)
        self.servo_writemicroseconds = rospy.ServiceProxy('ServoWritemicroseconds', rsrv.ServoWritemicroseconds)
        self.servo_read = rospy.ServiceProxy('ServoRead', rsrv.ServoRead)
        self.drive = 200

    def trymap(self, x, in_min, in_max, out_min, out_max):
        return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

    def calibrate()
    def trymicro(self):
        
        self.servo_read(0)
        '''
        us = self.trymap(self.drive, 0, 1000, 1000, 2000)
        self.servo_writemicroseconds(6, us)
        '''

if __name__ == "__main__":
    test().trymicro()
