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
import MadgwickAHRS
import MahonyAHRS
import MadgwickQuaternionLibrary

#variables area
cmd = "sraw"
VIEW_SIZE_X = 800
VIEW_SIZE_Y = 600
count = 0
grad2rad = 3.141592/180.0 # pi

fileData = []

def setup():
    size(VIEW_SIZE_X, VIEW_SIZE_Y)
    global tempEuler
    global euler

    global count
    count = 0
    global myFile
    myFile = open("test.txt", "r")

    euler = [0.0,0.0,0.0]  #// psi=x, theta=y, phi=z

    frameRate(10)

def read_from_file():
    global count;
    global myFile;
    global euler;

    if(count == 0):
        ## read whole file and keep data in fileData
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

        print "quaternion:";

        #MadgwickAHRS.MadgwickAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        MahonyAHRS.MahonyAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        #q = [MadgwickAHRS.q0,MadgwickAHRS.q1,MadgwickAHRS.q2,MadgwickAHRS.q3];
        q = [MahonyAHRS.q0,MahonyAHRS.q1,MahonyAHRS.q2,MahonyAHRS.q3];
        print q;

        print "euler:"
        MadgwickQuaternionLibrary.quaternionToEuler(q[0],q[1],q[2],q[3])
        euler[0] = MadgwickQuaternionLibrary.psi; # index 0 = psi = x
        euler[1] = MadgwickQuaternionLibrary.theta; # index 1 = theta = y
        euler[2] = MadgwickQuaternionLibrary.phi; # index 2 = phi = z
        print euler;

    count = count + 1



def serialEvent():
    global count
    global myFile
    global euler;

    #read_from_file()
    if(count == 0):
        ## read whole file and keep data in fileData
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

        print "quaternion:";

        MadgwickAHRS.MadgwickAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        #MahonyAHRS.MahonyAHRSupdate(float(line[1])*grad2rad,float(line[2])*grad2rad,float(line[3])*grad2rad,float(line[4]),float(line[5]),float(line[6]),float(line[7]),float(line[8]),float(line[9]))
        q = [MadgwickAHRS.q0,MadgwickAHRS.q1,MadgwickAHRS.q2,MadgwickAHRS.q3];
        #q = [MahonyAHRS.q0,MahonyAHRS.q1,MahonyAHRS.q2,MahonyAHRS.q3];
        print q;

        print "euler:"
        MadgwickQuaternionLibrary.quaternionToEuler(q[0],q[1],q[2],q[3])
        euler[0] = MadgwickQuaternionLibrary.psi; # index 0 = psi = x
        euler[1] = MadgwickQuaternionLibrary.theta; # index 1 = theta = y
        euler[2] = MadgwickQuaternionLibrary.phi; # index 2 = phi = z
        print euler;

    count = count + 1

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
    global euler;

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

