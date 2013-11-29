#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      nguyen
#
# Created:     06/03/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import numpy

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

def calibrate_raw_measures(gx, gy, gz, ax, ay, az, mx, my, mz):
    #global roll;  global pitch; global yaw;

    #global acc_dot
    #global acc_offset
    #global mag_dot
    #global mag_offset
    #global gyro_dot
    #global gyro_offset

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
