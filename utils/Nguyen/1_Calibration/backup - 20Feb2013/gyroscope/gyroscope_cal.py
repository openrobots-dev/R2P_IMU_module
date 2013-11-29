#-------------------------------------------------------------------------------
# Name:        gyroscope calibration
# Purpose:
#
# Author:      nguyen
#
# Created:     09/01/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import numpy
from numpy import linalg
import gyroscope_utils

def main():
    #----------------- 1.parameters for GYROSCOPE -----------------------------
    ref_speed_1 = 11417 # 33.3 rpm with sensitivity = 500dps -> 17.50 mdps/digit
    ref_speed_2 = 15428 # 45 rpm with sensitivity = 500dps -> 17.50 mdps/digit

    #files contain raw data
    file_z_33 = "data\\gyro_z_33.txt"
    file_z_45 = "data\\gyro_z_45.txt"
    file_y_33 = "data\\gyro_y_33.txt"
    file_y_45 = "data\\gyro_y_45.txt"
    file_x_33 = "data\\gyro_x_33.txt"
    file_x_45 = "data\\gyro_x_45.txt"
    #bias
    file_bias = "data\\gyro_bias.txt"

    # Step 1: Calculate bias - zero level
    meas_bias = gyroscope_utils.read_log(file_bias)
    offset = gyroscope_utils.calculate_gyro_bias(meas_bias)

    # ------------------------2. read raw measurements from log file ------------------------
    # measurements is an array of x,y,z
    #
    meas_z_33 = gyroscope_utils.read_log(file_z_33)
    meas_z_45 = gyroscope_utils.read_log(file_z_45)
    meas_y_33 = gyroscope_utils.read_log(file_y_33)
    meas_y_45 = gyroscope_utils.read_log(file_y_45)
    meas_x_33 = gyroscope_utils.read_log(file_x_33)
    meas_x_45 = gyroscope_utils.read_log(file_x_45)
    if (len(meas_z_33) == 0 or len(meas_z_45) == 0 or len(meas_y_33) == 0 or len(meas_y_45) == 0 or len(meas_x_33) == 0 or len(meas_x_45) == 0):
        print("Error: found zero in log file!")
        sys.exit(1)
    else:
       print("found "+str(len(meas_z_33))+" records after read log")

    # 3. construct raw measurement matrix and optimal matrix
    #x: real normalize smooth measures
    x = []
    #y: optimal (theoretical) value of normalize measures
    y = []
    gyroscope_utils.construct_real_ref_meas(x, y, meas_x_33, ref_speed_1, 0, 0, offset)
    gyroscope_utils.construct_real_ref_meas(x, y, meas_x_45, ref_speed_2, 0, 0, offset)
    gyroscope_utils.construct_real_ref_meas(x, y, meas_y_33, ref_speed_1, 1, 1, offset)
    gyroscope_utils.construct_real_ref_meas(x, y, meas_y_45, ref_speed_2, 1, 1, offset)
    gyroscope_utils.construct_real_ref_meas(x, y, meas_z_33, ref_speed_1, 2, 0, offset)
    gyroscope_utils.construct_real_ref_meas(x, y, meas_z_45, ref_speed_2, 2, 0, offset)

    print (len(x))
    print (len(y))

    x = numpy.array(x)
    y = numpy.array(y)

    #4. apply least square to compute 9 parameter from misalignment matrix and scale factors

    (v, residues, rank, shape) = linalg.lstsq(x, y)

    print v

    #convert raw params to calibration params
    calibration_params = gyroscope_utils.cal_calibration_params(v)
    #misalignment matrix
    misalignment_matrix = calibration_params[0]
    print "misalignment matrix:"
    print misalignment_matrix
    #scale factor matrix
    scale_factor = calibration_params[1]
    scale_factor_matrix = [[scale_factor[0], 0, 0],
                           [0, scale_factor[1], 0],
                           [0, 0, scale_factor[2]]]
    print "scale factors:"
    print scale_factor
    #offset
    print "offset:"
    print offset

    #write calibration params to file
    gyroscope_utils.write_cal_params_to_file("gyro_calibration_params.txt", misalignment_matrix, scale_factor_matrix, offset)

    #calculate calibrated measures
    #files contain calibrated data
    file_calibrated_z_33 = "data\\gyro_calibrated_z_33.txt"
    file_calibrated_z_45 = "data\\gyro_calibrated_z_45.txt"
    file_calibrated_y_33 = "data\\gyro_calibrated_y_33.txt"
    file_calibrated_y_45 = "data\\gyro_calibrated_y_45.txt"
    file_calibrated_x_33 = "data\\gyro_calibrated_x_33.txt"
    file_calibrated_x_45 = "data\\gyro_calibrated_x_45.txt"
    #bias
    file_calibrated_bias = "data\\gyro_calibrated_bias.txt"

    #z down
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_z_33, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_z_33, cal_data)
    #z up
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_z_45, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_z_45, cal_data)
    #y down
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_y_33, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_y_33, cal_data)
    #y up
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_y_45, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_y_45, cal_data)
    #x down
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_x_33, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_x_33, cal_data)
    #x up
    cal_data = gyroscope_utils.calculate_calibrated_meas(meas_x_45, misalignment_matrix, scale_factor_matrix, offset)
    gyroscope_utils.write_to_file(file_calibrated_x_45, cal_data)


if __name__ == "__main__":
    main()