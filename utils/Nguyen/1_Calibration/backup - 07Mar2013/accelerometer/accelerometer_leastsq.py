#-------------------------------------------------------------------------------
# Name:        accelerometer least square
# Purpose: This module will find calibration parameter by using least square method
#
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

    #2. low pass filter - not used
    #smooth_z_down = accelerometer_utils.lowpass_filter(meas_z_down)
    #smooth_z_up = accelerometer_utils.lowpass_filter(meas_z_up)
    #smooth_y_down = accelerometer_utils.lowpass_filter(meas_y_down)
    #smooth_y_up = accelerometer_utils.lowpass_filter(meas_y_up)
    #smooth_x_down = accelerometer_utils.lowpass_filter(meas_x_down)
    #smooth_x_up = accelerometer_utils.lowpass_filter(meas_x_up)
    ##plotting data after low pass filter
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

    #3. Normalize raw measurements to the range of [0-1]
    #x: real normalize smooth measures
    x = []
    #y: optimal value of normalize measures
    y = []
    accelerometer_utils.normalize_meas(x, y, smooth_z_down, 0)
    accelerometer_utils.normalize_meas(x, y, smooth_z_up, 1)
    accelerometer_utils.normalize_meas(x, y, smooth_y_down, 2)
    accelerometer_utils.normalize_meas(x, y, smooth_y_up, 3)
    accelerometer_utils.normalize_meas(x, y, smooth_x_down, 4)
    accelerometer_utils.normalize_meas(x, y, smooth_x_up, 5)

    print (len(x))
    print (len(y))

    x = numpy.array(x)
    y = numpy.array(y)

    #4. apply least square method to obtain parameter matrix
    (v, residues, rank, shape) = linalg.lstsq(x, y)
    print v
    #convert raw params to calibration params - extract misalignment matrix, scale factor matrix and offset vector
    calibration_params = accelerometer_utils.cal_calibration_params(v)
    misalignment_matrix = calibration_params[0]
    print "misalignment matrix:"
    print misalignment_matrix
    scale_factor = calibration_params[1]
    scale_factor_matrix = [[scale_factor[0], 0, 0],
                           [0, scale_factor[1], 0],
                           [0, 0, scale_factor[2]]]
    print "scale factors:"
    print scale_factor
    offset = calibration_params[2]
    print "offset:"
    print offset

    #5. write calibration params to file
    accelerometer_utils.write_cal_params_to_file("calibration_params.txt",misalignment_matrix,scale_factor_matrix,offset)

    #6. calculate calibrated measures
    #files contain calibrated data
    file_calibrated_z_down = "data\\acc_calibrated_z_down.txt"
    file_calibrated_z_up = "data\\acc_calibrated_z_up.txt"
    file_calibrated_y_down = "data\\acc_calibrated_y_down.txt"
    file_calibrated_y_up = "data\\acc_calibrated_y_up.txt"
    file_calibrated_x_down = "data\\acc_calibrated_x_down.txt"
    file_calibrated_x_up = "data\\acc_calibrated_x_up.txt"
    #z down
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_z_down, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_z_down, cal_data)
    #z up
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_z_up, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_z_up, cal_data)
    #y down
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_y_down, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_y_down, cal_data)
    #y up
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_y_up, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_y_up, cal_data)
    #x down
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_x_down, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_x_down, cal_data)
    #x up
    cal_data = accelerometer_utils.calculate_calibrated_meas(meas_x_up, misalignment_matrix, scale_factor_matrix, offset)
    accelerometer_utils.write_to_file(file_calibrated_x_up, cal_data)

if __name__ == "__main__":
    main()