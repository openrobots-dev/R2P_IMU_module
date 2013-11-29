#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      nguyen
#
# Created:     05/03/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import serial

#variables area
cmd = "sraw"
#attitude
roll = 0.0; pitch = 0.0; yaw = 0.0;

myPort = serial.Serial("COM3", baudrate = 115200, timeout = 0.1)
def openPort():
    if(not myPort.isOpen):
        myPort.open()
        myPort.flushInput()
        myPort.write("reset\r\n")
        sleep(1)
        myPort.write(cmd + " " + "100" + "\r\n")
        sleep(1)
        myPort.flushInput()


def readline():
    pass

