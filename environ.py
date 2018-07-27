import numpy as np
from Arduino import Arduino

DRIVE_PIN = 10
SENSOR_PIN = 0
ESC_PIN = 6


class Action():
    def __init__(self):
        self.drive_mag = 10
        self.upper_bound = 30
        self.lower_bound = -65
        
    
