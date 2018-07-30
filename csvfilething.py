import serial
import csv

serialInterface = '/dev/tty.wchusbserial1410'
arduino = serial.Serial( serialInterface , 115200, timeout=.1)

data = arduino.readline()[:2].decode("utf-8")

with open("test.txt", "a") as myfile:
    while data is not 'stop':
        if data:
            myfile.write(data)
        data = arduino.readline()[:-2].decode("utf-8") #the last bit gets rid of the new-line chars
