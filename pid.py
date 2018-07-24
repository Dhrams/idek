"""
Class that acts as a mutable struct
"""
class PIDStruct(object):
    def __init__(self, input_, Ki, Kp, Kd, oldError, dt, iState):
        self.input_ = input_
        self.Ki = Ki
        self.Kp = Kp
        self.Kd = Kd
        self.oldError = oldError
        self.dt = dt
        self.iState = iState

"""
class where the PID is implemented
"""
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

    def setup(self):
        # arduino.close()
        # arduino = serial.Serial('/dev/cu.wchusbserial1420', 115200)
        # board.Servos.attach(Esc_pin)
        # board.pinMode(10, "OUTPUT")
        # board.digitalWrite(10, "LOW")
        self.controller.input_ = self.angle_com
        self.controller.Kp = self.p_term
        self.controller.Ki = self.i_term
        self.controller.Kd = self.d_term
        self.controller.dt = 1.0/self.frequency
        # arduino.write_line("press any key to arm or c to calibrate")
        # while arduino.in_waiting && arduino.read():
        # while !arduino.in_waiting
        # if arduino.read().decode('utf-8').lower() == "c":
        #     calibrate(Esc_pin)
        # else:
        #     arm(Esc_pin)
    
    """
    Resets the PID controller to initialized state
    """
    
    def resetSystem(self):
        self.drive = 0
        self.updatedPid = False
        for i in range(0,self.buffersize):
            self.angle_com = 0
        self.controller.iState = 0
        self.controller.oldError = self.controller.input_ - self.angle_com
    
    """
    updates PID values as soon as anew pitch request is made
    
    inputs:
    com - pitch request
    
    returns:
    updatedPid - boolean for if the PID has been updated or not
    """
    def updatePID(self, com):
    
        """
        maps the given float to an integer value between out_min and out_max
        
        input:
        x - value to map
        in_min - min value that val is within, usually 0
        in_max - max value that val can be
        out_min - min value that val is to be mapped to
        out_max - max value that val is to be mapped to
        
        returns:
        mapped integer
        
        """
        def trymap(x, in_min, in_max, out_min, out_max):
            return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
        
        """
        constrains the value given to the range given
        
        input:
        val - the value to be constrained
        min_val - min value that val can be
        max_val - max valuse that val can be
        
        returns:
        value within the range given
        
        """
        def constrain(val, min_val, max_val):
            return min(max_val, max(min_val, val))
    
        pTerm, iTerm, dTerm, error = 0,0,0,0
        self.angle_com = com
        error = self.controller.input_ - self.angle_com
        pTerm = self.controller.Kp * error
        self.controller.iState += error * self.controller.dt
        self.controller.iState = constrain(self.controller.iState, self.min_i_term/self.controller.Ki, self.max_i_term/self.controller.Ki)
        iTerm = self.controller.Ki * self.controller.iState
        dTerm = self.controller.Kd * ((error - self.controller.oldError) / self.controller.dt)
        self.drive = pTerm + iTerm + dTerm
        # setSpeed(Esc_pin, self.drive)
        self.updatedPid = True
        return self.drive
