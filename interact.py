from Arduino import Arduino
import numpy as np

board = Arduino(port = "/dev/tty.wchusbserial...", 115200)
board.Servos.attach(6)



board.Servos.writeMicroseconds(trymap(drive))

filter(-0.3656*board.read(6) + 185.64)

