#-------------------------------------------------------------------------------
# Name:        gyroscope utils
# Purpose: contains common methods for gyroscope calibration
#
# Author:      nguyen
#
# Created:     10/01/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import re
import scipy
from scipy import linalg
from pylab import *
from mpl_toolkits.mplot3d import Axes3D
import numpy


#
# extracts raw sensor measurements from a log
#
def read_log(filename):
    f = open(filename, 'r')
    list_meas = []
    n = 0
    while True:
        n = n + 1
        line = f.readline().strip()
        if line == '':
            break
        line = line.split()
        #line[0] = timestamp
        list_meas.append([float(line[1]), float(line[2]), float(line[3])])
    return scipy.array(list_meas)

def calculate_gyro_bias(bias_meas):
    return numpy.mean(bias_meas, axis = 0)

#
#construct two matrix, one for real measure and one for reference measure at known angular rate
#construct in the form of AX = B where A=real_x, B
#                    PARAMS:
#real_x: stores real measure from sensor
#optimal_y: stores expected values at known angular rate
#raw_meas: raw measures collected from sensor
#ref_meas: expected values at known angular rate
#counterclockwise = 1: positive, counterclockwise = 0: negative
#offset: bias calculated in previous step
#
def construct_real_ref_meas(real_x, optimal_y, raw_meas, ref_meas, axis, counterclockwise, offset):
    offset_x = offset[0]
    offset_y = offset[1]
    offset_z = offset[2]
    for i in range(len(raw_meas)):
        row = raw_meas[i]
        x = row[0] - offset_x
        y = row[1] - offset_y
        z = row[2] - offset_z
        real_x.append([x,y,z])
        #construct optimal measure, number of rows = number of rows in real_x
        if (axis == 0) and (counterclockwise == 1): #x and rotate in counterclockwise -> ref_meas is positive
            optimal_y.append([ref_meas,0,0])
        elif (axis == 0) and (counterclockwise == 0): #x and rotate in clockwise -> negative
            optimal_y.append([-ref_meas,0,0])
        elif (axis == 1) and (counterclockwise == 1): #y and counterclockwise -> ref_meas is positive
            optimal_y.append([0,ref_meas,0])
        elif (axis == 1) and (counterclockwise == 0): #y and clockwise -> ref_meas is negative
            optimal_y.append([0,-ref_meas,0])
        elif (axis == 2) and (counterclockwise == 1): #z and counterclockwise -> ref_meas is positive
            optimal_y.append([0,0,ref_meas])
        elif (axis == 2) and (counterclockwise == 0): #z and clockwise -> ref_meas is negative
            optimal_y.append([0,0,-ref_meas])


#calculate directly 12-parameters matrix from the equation24: X = [w**T.w]**-1 . w**T . Y
def cal_raw_params(x,y):
    raw_params = numpy.dot(numpy.dot(linalg.inv(numpy.dot(numpy.transpose(x),x)), numpy.transpose(x)),y)
    return raw_params

#calculate calibration parameters from params obtained from raw 12-parameters matrix, obtained from direct calculation or least square
def cal_calibration_params(raw_params):
    # scale factors
    gyro11 = raw_params[0][0] # = 1/Sx
    gyro22 = raw_params[1][1] # = 1/Sy
    gyro33 = raw_params[2][2] # = 1/Sz
    scale_factors = [gyro11, gyro22, gyro33]
    #misalignment matrix
    gyro12 = raw_params[1][0]
    Mxy = gyro12/gyro22  #acc12 * Sy = acc12 / acc22
    gyro13 = raw_params[2][0]
    Mxz = gyro13/gyro33
    gyro21 = raw_params[0][1]
    Myx = gyro21/gyro11
    gyro23 = raw_params[2][1]
    Myz = gyro23/gyro33
    gyro31 = raw_params[0][2]
    Mzx = gyro31/gyro11
    gyro32 = raw_params[1][2]
    Mzy = gyro32/gyro22
    mis_matrix = [[1, Mxy, Mxz],
                  [Myx, 1, Myz],
                  [Mzx, Mzy, 1]]
    return [mis_matrix, scale_factors]


#
#calculate calibrated measure
#
def calculate_calibrated_meas(raw_meas, misalignment, scale, offset):
    cal_meas = []
    dot_mis_scale = numpy.dot(misalignment, scale)
    for i in range(len(raw_meas)):
        cal = numpy.dot(dot_mis_scale,numpy.subtract(raw_meas[i],offset))
        cal_meas.append(cal)
    return scipy.array(cal_meas)


#
# write to file
#
def write_to_file(filename, data):
    f = open(filename, 'w')
    for i in range(len(data)):
        line = str(data[i][0]) + " " + str(data[i][1]) + " " + str(data[i][2]) + "\n"
        f.write(line)
    f.close()


#
#write calibrate params to file
#
def write_cal_params_to_file(fileName, misalignment_matrix, scale_factor_matrix, offset):
    f = open(fileName,"w")
    f.write("\n misalignment matrix: \n")
    f.write(" [" + str(misalignment_matrix[0]) + "\n " + str(misalignment_matrix[1]) + "\n " + str(misalignment_matrix[2]) + "]")
    f.write("\n\n")
    f.write("\n scale factor matrix: \n")
    f.write(" [" + str(scale_factor_matrix[0]) + "\n " + str(scale_factor_matrix[1]) + "\n " + str(scale_factor_matrix[2]) + "]")
    f.write("\n\n")
    f.write("\n offset: \n")
    f.write(" [" + str(offset[0]) + " " + str(offset[1]) + " " + str(offset[2]) + "]")
    f.close()


