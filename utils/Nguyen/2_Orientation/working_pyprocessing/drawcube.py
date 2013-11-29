#-------------------------------------------------------------------------------
# Name:        draw cube
# Purpose:
#
# Author:      nguyen
#
# Created:     13/01/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
# Taken from FreeIMU_cube
#-------------------------------------------------------------------------------

from pyprocessing import *
import serial
from time import sleep
import numpy
import MadgwickQuaternionLibrary
import MadgwickAHRS as AHRS
#import MahonyAHRS as AHRS


#variables area
cmd = "sraw"
VIEW_SIZE_X = 800
VIEW_SIZE_Y = 600
count = 0
grad2rad = 3.141592/180.0 # pi

fileData = []

def setup():
    size(VIEW_SIZE_X, VIEW_SIZE_Y)
    global euler
    global acc_offset
    global acc_dot
    global mag_offset
    global mag_dot
    global gyro_offset
    global gyro_dot

    ##to read from file: uncomment this ----- DO NOT DELETE
    #global count
    #count = 0
    #global myFile
    #myFile = open("test.txt", "r")

    #to read from serial port
    global myPort
    myPort = serial.Serial("COM3", baudrate = 115200, timeout = 0.1)
    if(not myPort.isOpen):
        myPort.open()
    myPort.flushInput()
    myPort.write("reset\r\n")
    sleep(1)
    myPort.write(cmd + " " + "10" + "\r\n")
    sleep(1)
    myPort.flushInput()

    euler = [0.0,0.0,0.0]  #// psi=x, theta=y, phi=z

    #calibration constants
    #TODO: read calibration params from file
    acc_misalignment = [[1, -0.021071985722012798, 0.06286712108721447],
                        [0.066152630708061308, 1, -0.016468683329263566],
                        [-0.0074030016766424801, 0.062322196080008774, 1]]
    acc_scale_factor = [[0.99969281381684783, 0, 0],
                        [0, 0.99963729832296977, 0],
                        [0, 0, 1.0004635661247805]]

    acc_offset =  [0.0212962680227, 0.00451366611004, -0.0083203784396]

    mag_misalignment = [[0.974775, -0.222873, 0.011895],
                        [-0.082773, 0.996515, 0.010343],
                        [-0.086375, -0.162879, 0.982858]]
    mag_scale_factor = [[343.396264, 0, 0],
                        [0, 445.664929, 0],
                        [0, 0, 354.441437]]
    mag_offset = [-41, -84, 7]

    gyro_misalignment = [[1, -0.020743187766895724, -0.014461072652525046],
                         [0.012002952585861393, 1, -0.026537464314942662],
                         [0.0057692066318259551, -0.021160448720792441, 1]]
    gyro_scale_factor = [[1.0047217854157167, 0, 0],
                         [0, 0.96322237497132535, 0],
                         [0, 0, 0.97786838270837195]]
    gyro_offset = [-84.818, -106.672333333, 48.8486666667]

    #calculate dot product of misalignment matrix and scale factor matrix
    acc_dot = numpy.dot(acc_misalignment, acc_scale_factor)
    mag_dot = numpy.dot(mag_misalignment, mag_scale_factor)
    gyro_dot = numpy.dot(gyro_misalignment, gyro_scale_factor)

    frameRate(100)

def calibrate_raw_measures(gx, gy, gz, ax, ay, az, mx, my, mz):
    global acc_dot
    global acc_offset
    global mag_dot
    global mag_offset
    global gyro_dot
    global gyro_offset

    #calibrate gyroscope
    cal_gyro = numpy.dot(gyro_dot, numpy.subtract([gx, gy, gz], gyro_offset))
    cal_acc = numpy.dot(acc_dot, numpy.subtract([ax, ay, az], acc_offset))
    cal_mag = numpy.dot(mag_dot, numpy.subtract([mx, my, mz], mag_offset))
    gx = cal_gyro[0]
    gy = cal_gyro[1]
    gz = cal_gyro[2]
    ax = cal_acc[0]
    ay = cal_acc[1]
    az = cal_acc[2]
    mx = cal_mag[0]
    my = cal_mag[1]
    mz = cal_mag[2]
    return (gx, gy, gz, ax, ay, az, mx, my, mz)

def read_from_file():
    global count
    global myFile

    ## read whole file and keep data in fileData
    if(count == 0):
        for i, lineF in enumerate(myFile):
            fileData.append(lineF)

    if(count >= len(fileData)):
        print "out of file"
        return

    ## get current line
    line = fileData[count]
    if(len(line) <> 0):
        print "data line:"
        line = line.split()
        print line;

        line = calibrate_raw_measures(int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))

        print "quaternion:";
        #MadgwickAHRSupdate
        #raw data - without calibration
        #AHRS.MadgwickAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        #calibrated data
        AHRS.MadgwickAHRSupdate(float(line[0])*grad2rad,float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3]),float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]))

        #MahonyAHRSupdate
        #raw data - without calibration
        #AHRS.MahonyAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        #calibrated data
        #AHRS.MahonyAHRSupdate(float(line[0])*grad2rad,float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3]),float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]))

        q = [AHRS.q0,AHRS.q1,AHRS.q2,AHRS.q3];
        print q;

        print "euler:"
        MadgwickQuaternionLibrary.quaternionToEuler(q[0],q[1],q[2],q[3])
        euler[0] = MadgwickQuaternionLibrary.psi; # index 0 = psi = x
        euler[1] = MadgwickQuaternionLibrary.theta; # index 1 = theta = y
        euler[2] = MadgwickQuaternionLibrary.phi; # index 2 = phi = z
        print euler;

    count = count + 1


def read_from_port():
    global myPort
    line = myPort.readline()
    line = line.split()

    print ("raw data: " + str(line))

    line = calibrate_raw_measures(int(line[1]),int(line[2]),int(line[3]),int(line[4]),int(line[5]),int(line[6]),int(line[7]),int(line[8]),int(line[9]))

    #print "quaternion:";
    #MadgwickAHRSupdate
    #raw data - without calibration
    #AHRS.MadgwickAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
    #calibrated data
    AHRS.MadgwickAHRSupdate(float(line[0])*grad2rad,float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3]),float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]))

    #MahonyAHRSupdate
    #raw data - without calibration
    #AHRS.MahonyAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
    #calibrated data
    #AHRS.MahonyAHRSupdate(float(line[0])*grad2rad,float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3]),float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]))

    q = [AHRS.q0,AHRS.q1,AHRS.q2,AHRS.q3];
    #print q;

    #print "euler:"
    MadgwickQuaternionLibrary.quaternionToEuler(q[0],q[1],q[2],q[3])
    euler[0] = MadgwickQuaternionLibrary.psi; # index 0 = psi = x
    euler[1] = MadgwickQuaternionLibrary.theta; # index 1 = theta = y
    euler[2] = MadgwickQuaternionLibrary.phi; # index 2 = phi = z
    #print euler;

def serialEvent():
    #read_from_file()
    read_from_port()


def buildBoxShape():
    # box(60, 10, 40);
    noStroke();
    beginShape(QUADS);

    # Z+ (to the drawing area)
    fill(255,250)
    vertex(-30, -5, 20);
    vertex(30, -5, 20);
    vertex(30, 5, 20);
    vertex(-30, 5, 20);

    #//Z-
    fill(240, 235);
    vertex(-30, -5, -20);
    vertex(30, -5, -20);
    vertex(30, 5, -20);
    vertex(-30, 5, -20);

    #//X-
    fill(230, 225);
    vertex(-30, -5, -20);
    vertex(-30, -5, 20);
    vertex(-30, 5, 20);
    vertex(-30, 5, -20);

    #//X+
    fill(220,215);
    vertex(30, -5, -20);
    vertex(30, -5, 20);
    vertex(30, 5, 20);
    vertex(30, 5, -20);

    #//Y-
    fill(210,205);
    vertex(-30, -5, -20);
    vertex(30, -5, -20);
    vertex(30, -5, 20);
    vertex(-30, -5, 20);

    #//Y+
    fill(200,195);
    vertex(-30, 5, -20);
    vertex(30, 5, -20);
    vertex(30, 5, 20);
    vertex(-30, 5, 20);

    endShape();


def drawCube():
    pushMatrix();
    translate(VIEW_SIZE_X/2, VIEW_SIZE_Y/2 + 50, 0);
    scale(5,5,5);

    # a demonstration of the following is at
    # http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
    rotateZ(-euler[2]); # phi = z
    rotateX(-euler[1]); # theta = y
    rotateY(-euler[0]); # psi = x

    buildBoxShape();
    popMatrix();


def draw():
    global euler

    background(0,0,0);
    fill(255,255,255);

    text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);

    text("Euler Angles:\nYaw (psi)  : " + str(degrees(euler[0])) + "\nPitch (theta): " + str(degrees(euler[1])) + "\nRoll (phi)  : " + str(degrees(euler[2])), 200, 20);
    drawCube()
    serialEvent()

run()

