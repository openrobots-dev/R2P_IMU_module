#-------------------------------------------------------------------------------
# Name:        accelerometer solver
# Purpose: This module solve the calibration parameter directly using equation: Y = w.X
#           where Y is expected normalized value, X is normalize mean-value of raw measurements
# Author:      nguyen
#
# Created:     02/01/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import sys
import os
from optparse import OptionParser
import scipy
from scipy import optimize
import numpy
from numpy import linalg

import accelerometer_utils

def main():

    #files contain raw data
    file_z_down = "data\\acc_z_down.txt"
    file_z_up = "data\\acc_z_up.txt"
    file_y_down = "data\\acc_y_down.txt"
    file_y_up = "data\\acc_y_up.txt"
    file_x_down = "data\\acc_x_down.txt"
    file_x_up = "data\\acc_x_up.txt"

    # ------------------------1. read raw measurements from log file ------------------------
    # measurements is an array of x,y,z
    #
    meas_z_down = accelerometer_utils.read_log(file_z_down)
    meas_z_up = accelerometer_utils.read_log(file_z_up)
    meas_y_down = accelerometer_utils.read_log(file_y_down)
    meas_y_up = accelerometer_utils.read_log(file_y_up)
    meas_x_down = accelerometer_utils.read_log(file_x_down)
    meas_x_up = accelerometer_utils.read_log(file_x_up)
    if (len(meas_z_down) == 0 or len(meas_z_up) == 0 or len(meas_y_down) == 0 or len(meas_y_up) == 0 or len(meas_x_down) == 0 or len(meas_x_up) == 0):
        print("Error: found zero in log file!")
        sys.exit(1)
    else:
       print("found "+str(len(meas_z_down))+" records after read log")

    ##2. low pass filter --> RIGHT NOW WE DONT USE IT
    #smooth_z_down = accelerometer_utils.lowpass_filter(meas_z_down)
    #smooth_z_up = accelerometer_utils.lowpass_filter(meas_z_up)
    #smooth_y_down = accelerometer_utils.lowpass_filter(meas_y_down)
    #smooth_y_up = accelerometer_utils.lowpass_filter(meas_y_up)
    #smooth_x_down = accelerometer_utils.lowpass_filter(meas_x_down)
    #smooth_x_up = accelerometer_utils.lowpass_filter(meas_x_up)
    #if(len(smooth_z_down) == 0 or len(smooth_z_up) == 0 or len(smooth_y_down) == 0 or len(smooth_y_up) == 0 or len(smooth_x_down) == 0 or len(smooth_x_up) == 0):
    #    print("Error: found zero after low pass filter.")
    #    sys.exit()
    #else:
    #    print ("found" + str(len(smooth_z_down)) + " after low pass filter.")
    #    accelerometer_utils.plot_lowpass_filter(True, meas_z_up, smooth_z_up)
    smooth_z_down = meas_z_down
    smooth_z_up = meas_z_up
    smooth_y_down = meas_y_down
    smooth_y_up = meas_y_up
    smooth_x_down = meas_x_down
    smooth_x_up = meas_x_up


    #3. calculate parameter matrix directly
    # Optimal value of y
    y = [[0, 0, 1],
         [0, 0, -1],
         [0, 1, 0],
         [0, -1, 0],
         [1, 0, 0],
         [-1, 0, 0]]

    #normalize smooth measures
    x = []
    norm = accelerometer_utils.normalize_mean_meas(smooth_z_down)
    x.append([norm[0], norm[1], norm[2], 1])
    norm = accelerometer_utils.normalize_mean_meas(smooth_z_up)
    x.append([norm[0], norm[1], norm[2], 1])
    norm = accelerometer_utils.normalize_mean_meas(smooth_y_down)
    x.append([norm[0], norm[1], norm[2], 1])
    norm = accelerometer_utils.normalize_mean_meas(smooth_y_up)
    x.append([norm[0], norm[1], norm[2], 1])
    norm = accelerometer_utils.normalize_mean_meas(smooth_x_down)
    x.append([norm[0], norm[1], norm[2], 1])
    norm = accelerometer_utils.normalize_mean_meas(smooth_x_up)
    x.append([norm[0], norm[1], norm[2], 1])

    #calculate directly
    raw_params = accelerometer_utils.cal_raw_params(x, y) # x is raw measures, y is expected measures (both are normalized)
    calibration_params = accelerometer_utils.cal_calibration_params(raw_params)
    print "misalignment matrix:"
    print calibration_params[0]
    print "scale factors:"
    print calibration_params[1]
    print "offset:"
    print calibration_params[2]


if __name__ == "__main__":
    main()