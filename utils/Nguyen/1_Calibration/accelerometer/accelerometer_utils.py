#-------------------------------------------------------------------------------
# Name:        accelerometer_utils
# Purpose: Contains several utility functions for calibration, sharing among several modules
#
# Author:      Nguyen Ho Thi Thao (email: nguyen.ho@mail.polimi.it)
#
# Created:     02/01/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

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
    #while n < 15:
        n = n + 1
        line = f.readline().strip()
        if line == '':
            break
        #line = f.readline()
        line = line.split()
        #line[0] = timestamp
        list_meas.append([float(line[1]), float(line[2]), float(line[3])])
    return scipy.array(list_meas)

#low pass filter
def lowpass_filter(acceleration):
    x0 = 0
    y0 = 0
    z0 = 0
    smoothed = []

    dt = (1.0 / 10)
    RC = 0.2
    alpha = dt / (RC + dt)

    for i in range(len(acceleration)):
        acc = acceleration[i]
        x0 = (alpha * acc[0]) + (1.0 - alpha) * x0
        y0 = (alpha * acc[1]) + (1.0 - alpha) * y0
        z0 = (alpha * acc[2]) + (1.0 - alpha) * z0
        smoothed.append([x0,y0,z0])
    return smoothed

#
#normalize the mean meansurement for each position of axis (z down/up, y down/up, x down/up)
#
def normalize_mean_meas(smooth_meas):
    #1. take the means for each files
    meas_mean = numpy.mean(smooth_meas, axis = 0)
    #2. normalize
    meas_norm = linalg.norm(meas_mean)
    meas_mean = meas_mean / meas_norm
    return meas_mean

#
#normalize the raw measurements
#
def normalize_meas(real_x, optimal_y,smooth_meas, direction):
    for i in range(len(smooth_meas)):
        #normalize raw measures
        row = smooth_meas[i]
        row_norm = linalg.norm(row)
        x = row[0]/row_norm
        y = row[1]/row_norm
        z = row[2]/row_norm
        real_x.append([x,y,z,1])
        #construct optimal measure, number of rows = number of rows in real_x
        if direction == 0: #z_down
            optimal_y.append([0,0,1])
        elif direction == 1: #z_up
            optimal_y.append([0,0,-1])
        elif direction == 2: #y_down
            optimal_y.append([0,1,0])
        elif direction == 3: #y_up
            optimal_y.append([0,-1,0])
        elif direction == 4: #x_down
            optimal_y.append([1,0,0])
        elif direction == 5: #x_up
            optimal_y.append([-1,0,0])

#calculate directly 12-parameters matrix from the equation24: X = [w**T.w]**-1 . w**T . Y
def cal_raw_params(x,y):
    raw_params = numpy.dot(numpy.dot(linalg.inv(numpy.dot(numpy.transpose(x),x)), numpy.transpose(x)),y)
    return raw_params

#calculate calibration parameters from params obtained from raw 12-parameters matrix, obtained from direct calculation or least square
def cal_calibration_params(raw_params):
    # scale factors
    acc11 = raw_params[0][0] # = 1/Sx
    acc22 = raw_params[1][1] # = 1/Sy
    acc33 = raw_params[2][2] # = 1/Sz
    scale_factors = [acc11, acc22, acc33]
    #misalignment matrix
    acc12 = raw_params[1][0]
    Mxy = acc12/acc22  #acc12 * Sy = acc12 / acc22
    acc13 = raw_params[2][0]
    Mxz = acc13/acc33
    acc21 = raw_params[0][1]
    Myx = acc21/acc11
    acc23 = raw_params[2][1]
    Myz = acc23/acc33
    acc31 = raw_params[0][2]
    Mzx = acc31/acc11
    acc32 = raw_params[1][2]
    Mzy = acc32/acc22
    mis_matrix = [[1, Mxy, Mxz],
                  [Myx, 1, Myz],
                  [Mzx, Mzy, 1]]
    #offset
    acc10 = raw_params[3][0]
    acc20 = raw_params[3][1]
    acc30 = raw_params[3][2]
    left_matrix = [[acc11, acc12, acc13],
                   [acc21, acc22, acc23],
                   [acc31, acc32, acc33]]
    offset = calculate_offset(left_matrix,[acc10,acc20,acc30])
    return [mis_matrix, scale_factors, offset]

#
# calculate offset by solving equation: y = Mx
#
def calculate_offset(left_matrix, y):
    offset = linalg.solve(left_matrix,y)
    return offset


#
#calculate calibrated measure
#
def calculate_calibrated_meas(raw_meas, misalignment, scale, offset):
    cal_meas = []
    dot_mis_scale = numpy.dot(misalignment, scale)
    for i in range(len(raw_meas)):
        cal = numpy.dot(dot_mis_scale,numpy.subtract(raw_meas[i],offset))
        #cal_norm = linalg.norm(cal)
        #a = cal / cal_norm
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
    f.write(" [" + str(offset[0]) + ", " + str(offset[1]) + ", " + str(offset[2]) + "]")
    f.close()



