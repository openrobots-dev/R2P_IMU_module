#!/usr/bin/python

import sys
import serial
from time import sleep
from datetime import datetime

if(len(sys.argv) < 2):
	print "Usage: " + sys.argv[0] + " IN_TTY BAUDRATE [filename] [frequency] [samples]"
	sys.exit()

if(len(sys.argv) > 3):
    filename = sys.argv[3]
else:
    filename = "log.txt";

if(len(sys.argv) > 4):
    frequency = int(sys.argv[4])
else:
	frequency = 10


if(len(sys.argv) > 5):
    samples = int(sys.argv[5])
else:
	samples = 100

buffer = ""
count = 0

f = open(filename, 'w')

serial_port = serial.Serial(sys.argv[1], baudrate=sys.argv[2], timeout=0.1)

serial_port.flushInput()

serial_port.write("reset\r\n")
sleep(1)
serial_port.write("sraw " + str(frequency) + "\r\n")
sleep(1)
serial_port.flushInput()

while count < samples :
    line = serial_port.readline()
    print(line)
    f.write(line)
    count = count + 1
	
serial_port.close()
f.close()
