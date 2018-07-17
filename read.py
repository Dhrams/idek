import serial
from collections import namedtuple
from Arduino import Arduino

# arduino = serial.Serial('/dev/cu.wchusbserial1420', 115200)
board = Arduino('115200')
class PIDStruct():
    def __init__(self, input: float, Ki: float, Kp: float, Kd: float, oldError: float, dt: float, iState: float):
        self.input = input
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd
        self.oldError = oldError
        self.dt = dt
        self.iState = iState

class PID(object):
    def __init__(self, p_term, i_term, d_term, angle_com):
        self.p_term = p_term
        self.i_term = i_term
        self.d_term = d_term
        self.controller = PIDStruct(0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00)
        self.min_i_term = -250
        self.max_i_term = 250
        self.angle_com = angle_com
        self.frequency = 100
        self.minAngle = -65
        self.maxAngle = 30
        self.maxFrequency = 1000
        self.buffersize = 2
        self.filteredVal = 0
        self.drive = 0
        self.index = 0
        self.updatedPid = False
        self.filterBuffer = [None] * self.buffersize

    def constrain(val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def filter(self, value):
        self.filteredVal -= self.filterBuffer[self.index] / self.buffersize
        self.filterBuffer[self.index] = value
        self.index+=1
        self.filteredVal += value / self.buffersize
        self.index %= self.buffersize
        return self.filteredVal

    def resetSystem(self):
        self.drive = 0
        self.updatedPid = False
        for i in range(0,self.buffersize):
            self.angle_com = filter((-.3656 * board.analogRead()) + 185.64)
        self.controller.iState = 0
        self.controller.oldError = self.controller.input - self.angle_com

    def updatePID(self):
        pTerm, iTerm, dTerm, error = 0
        self.angle_com = filter((-.3656 * arduino.read()) + 185.64)
        error = self.controller.input - self.angle_com
        pTerm = self.controller.Kp * error
        self.controller.iState += error * self.controller.dt
        self.controller.iState = constrain(self.controller.iState, self.min_i_term/self.controller.Ki, self.max_i_term/self.controller.Ki)
        iTerm = self.controller.Ki * self.controller.iState
        dTerm = self.controller.Kd * ((error - self.controller.oldError) / self.controller.dt)
        self.drive = pTerm + iTerm + dTerm
        self.updatedPid = True

    def arm(self, ESC):


"""
while True:
    command = str(input ("Command: ")).encode('utf-8')       # query servo position
    arduino.write(command)                          # write position to serial port
    reachedPos = str(arduino.readline())            # read serial port for arduino echo
    print(reachedPos)
"""
