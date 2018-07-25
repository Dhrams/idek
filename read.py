import pid as p

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

"""
Not really a simulation I dont think, idk anymore tbh

inputs:
theta - desired angle

returns:
Prints pitch, drive, and angle given

"""



def simulation(theta, pitch):
    pid = p.PID(3,1.5,.4,0)
    pid.setup()
    if theta >= pid.minAngle & theta <= pid.maxAngle:
        pid.controller.oldError = theta - pid.angle_com
        pid.controller.input_ = theta
        pid.updatePID(pitch)
    if pid.updatedPid:
        print(pid.angle_com)
        print("\t")
        print(pid.drive)
        print("\t")
        print(pid.controller.input_)
        pid.updatedPid = False


while True:
    angle = int(input("Give an angle: "))
    pitch = int(input("give a pitch: "))
    simulation(angle, pitch)
