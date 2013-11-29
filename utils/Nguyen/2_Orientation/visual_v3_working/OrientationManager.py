#--------------------------------------------------------------------------------------------------------------
# Name:        Orientation Manager
# Purpose: Control orientation mode (orientation from serial port, file) and algorithm used (Madgwick, Mahony)
#
# Author:      Nguyen Ho Thi Thao (nguyen.ho@mail.polimi.it)
#
# Created:     26/02/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#--------------------------------------------------------------------------------------------------------------

import serial
import MadgwickAHRS
import MahonyAHRS
import CalibrationManager
from time import sleep

#specify read mode: from file (=1), from serial port (=0)
read_from_file = 0
#specify calibration mode: do not calibrate (=0), calibrate (=1)
need_calibrate = 0
#specify the filtering algorithm: Madgwick (=1), Mahony (=0)
AHRS_Madgwick_algorithm = 1

#serial port
serialPort = "/dev/ttyUSB0"
#command to send
cmd = "sraw" #--stream all the sensors
#sample frequency
sampleRate = 100

#from degree to radian
grad2rad = 3.141592/180.0
#Euler angels
roll = 0.0; pitch = 0.0; yaw = 0.0;

#connect to serial port and send command to stream data from sensors
def openPort():
    global myPort
    myPort = serial.Serial(serialPort, baudrate = 115200, timeout = 0.1)
    if(not myPort.isOpen):
        myPort.open()
    myPort.flushInput()
    myPort.write("reset\r\n")
    sleep(1)
    myPort.write(cmd + " " + str(sampleRate) + "\r\n")
    sleep(1)
    myPort.flushInput()

#open file contains data and initialize variables
def openFile():
    #to read from file
    global myFile; global fileData; global lineNumberOfFile;
    myFile = open("test.txt", "r") #data_madgwick_calibrated.txt
    fileData = []
    lineNumberOfFile = 0;

#read Euler angels directly from file - TODO
def readDirectEuler():
    pass

#read sensor raw data from file
def readFromFile():
    global lineNumberOfFile;
    global myFile

    # read whole file and keep data in fileData
    if(lineNumberOfFile == 0):
        for i, lineF in enumerate(myFile):
            fileData.append(lineF)

    if(lineNumberOfFile >= len(fileData)):
        print "out of file"
        return

    # get current line
    line = fileData[lineNumberOfFile]
    if(len(line) <> 0):
        lineNumberOfFile = lineNumberOfFile + 1
        line = line.split()

        if(need_calibrate == 1): #calibrated data
            line = CalibrationManager.calibrate_raw_measures(int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))
        else:
            line = (int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))

        #convert milidegree to degree
        gx = float(line[0])*0.0175; gy = float(line[1])*0.0175; gz = float(line[2])*0.0175;
        #convert mili-g to g
        ax = float(line[3])/1000; ay = float(line[4])/1000; az = float(line[5])/1000;
        #mx = float(line[6]); my = float(line[7]); mz = float(line[8]);
        mx = (float(line[6]) - (-440.0)) / (510 - (-440)) * 2 - 1.0; my = (float(line[7]) - (-740.0)) / (380 - (-740)) * 2 - 1.0; mz = (float(line[8]) - (-500.0)) / (500 - (-500)) * 2 - 1.0;

        if(AHRS_Madgwick_algorithm == 1):
            (q0, q1, q2, q3) = MadgwickAHRS.MadgwickAHRSupdate(-(gx * grad2rad), gy * grad2rad, -(gz * grad2rad), -ax, ay, az, mx, -my, -mz);
            attitude_data = MadgwickAHRS.getMadAttitude();
        else:
            (q0, q1, q2, q3) = MahonyAHRS.MahonyAHRSupdate(-(gx * grad2rad), gy * grad2rad, -(gz * grad2rad), -ax, ay, az, mx, -my, -mz);
            attitude_data = MahonyAHRS.getMahAttitude();

        roll = attitude_data[0];
        pitch = attitude_data[1];
        yaw = attitude_data[2];
        return [roll, pitch, yaw];


def readFromPort():
    global myPort;
    line = myPort.readline()
    line = line.split()

    if(need_calibrate == 1): #calibrated data
        line = CalibrationManager.calibrate_raw_measures(int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))
    else:
        line = (int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))

    gx = float(line[0])*0.0175; gy = float(line[1])*0.0175; gz = float(line[2])*0.0175;
    ax = float(line[3])/1000; ay = float(line[4])/1000; az = float(line[5])/1000;
    mx = float(line[6]); my = float(line[7]); mz = float(line[8]);
    #mx = (float(line[6]) - (-440.0)) / (510 - (-440)) * 2 - 1.0; my = (float(line[7]) - (-740.0)) / (380 - (-740)) * 2 - 1.0; mz = (float(line[8]) - (-500.0)) / (500 - (-500)) * 2 - 1.0;

    if(AHRS_Madgwick_algorithm == 1):
        (q0, q1, q2, q3) = MadgwickAHRS.MadgwickAHRSupdate(-(gx * grad2rad), gy * grad2rad, -(gz * grad2rad), -ax, ay, az, mx, -my, -mz);
        attitude_data = MadgwickAHRS.getMadAttitude();
    else:
        (q0, q1, q2, q3) = MahonyAHRS.MahonyAHRSupdate(-(gx * grad2rad), gy * grad2rad, -(gz * grad2rad), -ax, ay, az, mx, -my, -mz);
        attitude_data = MahonyAHRS.getMahAttitude();

    roll = attitude_data[0];
    pitch = attitude_data[1];
    yaw = attitude_data[2];
    return [roll, pitch, yaw];

def initialize():
    if(read_from_file == 0):
        openPort()
    elif(read_from_file == 1):
        openFile()

def readline():
    attitude = [0.0, 0.0, 0.0]
    if(read_from_file == 0):
        attitude = readFromPort()
    elif (read_from_file == 1):
        attitude = readFromFile()
    elif(read_from_file == 2):
        attitude = readDirectEuler()

    return attitude

