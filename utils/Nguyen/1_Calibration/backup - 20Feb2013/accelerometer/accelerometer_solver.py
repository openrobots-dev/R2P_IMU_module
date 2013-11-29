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
    #----------------- 1.parameters for ACCELEROMETER -----------------------------
    sensor_ref = 9.81
    sensor_res = 10
    noise_window = 20; # noise_window used to eliminate first and last noise_window rows
    noise_threshold = 40; # noise_threshold: max standard deviation value of data

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

    #2. low pass filter
    smooth_z_down = accelerometer_utils.lowpass_filter(meas_z_down)
    smooth_z_up = accelerometer_utils.lowpass_filter(meas_z_up)
    smooth_y_down = accelerometer_utils.lowpass_filter(meas_y_down)
    smooth_y_up = accelerometer_utils.lowpass_filter(meas_y_up)
    smooth_x_down = accelerometer_utils.lowpass_filter(meas_x_down)
    smooth_x_up = accelerometer_utils.lowpass_filter(meas_x_up)
    if(len(smooth_z_down) == 0 or len(smooth_z_up) == 0 or len(smooth_y_down) == 0 or len(smooth_y_up) == 0 or len(smooth_x_down) == 0 or len(smooth_x_up) == 0):
        print("Error: found zero after low pass filter.")
        sys.exit()
    else:
        print ("found" + str(len(smooth_z_down)) + " after low pass filter.")
        accelerometer_utils.plot_lowpass_filter(True, meas_z_up, smooth_z_up)

    #3. filter out noisy measurements by cutting the samples that have large standard deviation
    # right now we will not use it.
    #flt_meas, flt_idx = accelerometer_utils.filter_meas(measurements, noise_window, noise_threshold)
    #print("remaining " + str(len(flt_meas)) + " after low pass")
    #if len(flt_meas) == 0:
    #    print("Error: found zero IMU in log file after low pass!") #+options.sensor+"_RAW measurements for aircraft with id "+options.ac_id+" in log file after low pass!")
    #    sys.exit(1)

    #4. parameters declare
    # Parametric function: 'v' is the parameter vector, 'x' the independent varible
    fp = lambda v, x: numpy.dot(x, v)  # v[0]/(x**v[1])*sin(v[2]*x)
    # Error function
    e = lambda v, x, y: (fp(v,x)-y)
    # Initial parameter value
    v0 = [[1, 0, 0],
          [0, 1, 0],
          [0, 0, 1],
          [1, 1, 1]]
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
    ## Fitting
    #v, success = optimize.leastsq(e, v0, args=(x,y))

    ## get an initial min/max guess
    #p0 = accelerometer_utils.get_min_max_guess(flt_meas, sensor_ref)
    #cp0, np0 = accelerometer_utils.scale_measurements(flt_meas, p0)
    #print("initial guess : avg "+str(np0.mean())+" std "+str(np0.std()))

    #def err_func(p, meas, y):
    #    cp, np = accelerometer_utils.scale_measurements(meas, p)
    #    err = y*scipy.ones(len(meas)) - np
    #    return err

    #p1, success = optimize.leastsq(err_func, p0[:], args=(flt_meas, sensor_ref))
    #cp1, np1 = accelerometer_utils.scale_measurements(flt_meas, p1)

    #print("optimized guess : avg " + str(np1.mean()) + " std " + str(np1.std()))

    #accelerometer_utils.print_xml(p1, options.sensor, sensor_res)
    #print("")

    ## otherwise show the first plot (blocking)
    #accelerometer_utils.plot_results(True, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref)


if __name__ == "__main__":
    main()