#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      nguyen
#
# Created:     26/02/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import serial
import MadgwickAHRS
import MahonyAHRS
import CalibrationManager
from time import sleep

#flag
read_from_file = 0
need_calibrate = 1
AHRS_Madgwick_algorithm = 1

#serial port

cmd = "sraw"
sampleRate = 50

grad2rad = 3.141592/180.0
#Euler angels
roll = 0.0; pitch = 0.0; yaw = 0.0;


def openPort():
    global myPort
    myPort = serial.Serial("COM3", baudrate = 115200, timeout = 0.1)
    if(not myPort.isOpen):
        myPort.open()
    myPort.flushInput()
    myPort.write("reset\r\n")
    sleep(1)
    myPort.write(cmd + " " + str(sampleRate) + "\r\n")
    sleep(1)
    myPort.flushInput()

def openFile():
    #to read from file
    global myFile; global fileData; global lineNumberOfFile;
    myFile = open("test.txt", "r") #data_madgwick_calibrated.txt
    fileData = []
    lineNumberOfFile = 0;

def readDirectEuler():
    pass

def readFromFile():
    global lineNumberOfFile;
    #global roll;  global pitch; global yaw;
    global myFile

    ## read whole file and keep data in fileData
    if(lineNumberOfFile == 0):
        for i, lineF in enumerate(myFile):
            fileData.append(lineF)

    if(lineNumberOfFile >= len(fileData)):
        print "out of file"
        return

    ## get current line
    line = fileData[lineNumberOfFile]
    if(len(line) <> 0):
        lineNumberOfFile = lineNumberOfFile + 1
        line = line.split()

        if(need_calibrate == 1): #calibrated data
            line = CalibrationManager.calibrate_raw_measures(int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))
        else:
            line = (int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))

        gx = float(line[0]); gy = float(line[1]); gz = float(line[2]);
        ax = float(line[3])/1000; ay = float(line[4])/1000; az = float(line[5])/1000;
        mx = float(line[6]); my = float(line[7]); mz = float(line[8]);

        ax=0.0;ay=0.0;az=1.0;
        gx=0.0;gy=0.0;gz=gz*0.0175;
        mx=0.0;my=0.0;mz=0.0;

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

    #ax=0.0;ay=0.0;az=1.0;
    #gx=0.0;gy=0.0;gz=gz*0.0175;
    #mx=0.0;my=0.0;mz=0.0;
    #print (ax, ay, az)
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

