#-------------------------------------------------------------------------------
# Name:        Calibration Manager
# Purpose: calculate calibrated data
#
# Author:      nguyen
#
# Created:     27/02/2013
# Copyright:   (c) nguyen 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import numpy
import MadgwickAHRS as AHRS
#import MahonyAHRS as AHRS

class OrientationManager:

    def __init__(self):
        #calibration constants
        #TODO: read calibration params from file
        self.acc_misalignment = [[1, -0.021071985722012798, 0.06286712108721447],
                            [0.066152630708061308, 1, -0.016468683329263566],
                            [-0.0074030016766424801, 0.062322196080008774, 1]]
        self.acc_scale_factor = [[0.99969281381684783, 0, 0],
                            [0, 0.99963729832296977, 0],
                            [0, 0, 1.0004635661247805]]

        self.acc_offset =  [0.0212962680227, 0.00451366611004, -0.0083203784396]

        self.mag_misalignment = [[0.974775, -0.222873, 0.011895],
                            [-0.082773, 0.996515, 0.010343],
                            [-0.086375, -0.162879, 0.982858]]
        self.mag_scale_factor = [[343.396264, 0, 0],
                            [0, 445.664929, 0],
                            [0, 0, 354.441437]]
        self.mag_offset = [-41, -84, 7]

        self.gyro_misalignment = [[1, -0.020743187766895724, -0.014461072652525046],
                             [0.012002952585861393, 1, -0.026537464314942662],
                             [0.0057692066318259551, -0.021160448720792441, 1]]
        self.gyro_scale_factor = [[1.0047217854157167, 0, 0],
                             [0, 0.96322237497132535, 0],
                             [0, 0, 0.97786838270837195]]
        self.gyro_offset = [-84.818, -106.672333333, 48.8486666667]

        #calculate dot product of misalignment matrix and scale factor matrix
        self.acc_dot = numpy.dot(self.acc_misalignment, self.acc_scale_factor)
        self.mag_dot = numpy.dot(self.mag_misalignment, self.mag_scale_factor)
        self.gyro_dot = numpy.dot(self.gyro_misalignment, self.gyro_scale_factor)

        #attitude
        self.roll = 0.0;
        self.pitch = 0.0;
        self.yaw = 0.0;

        #constant
        self.grad2rad = 3.141592/180.0 # pi


    def calibrate_raw_measures(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        global acc_dot
        global acc_offset
        global mag_dot
        global mag_offset
        global gyro_dot
        global gyro_offset

        #calibrate gyroscope
        cal_gyro = numpy.dot(self.gyro_dot, numpy.subtract([gx, gy, gz], self.gyro_offset))
        cal_acc = numpy.dot(self.acc_dot, numpy.subtract([ax, ay, az], self.acc_offset))
        cal_mag = numpy.dot(self.mag_dot, numpy.subtract([mx, my, mz], self.mag_offset))
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

    def quaternionUpdate(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        #MadgwickAHRSupdate
        AHRS.MadgwickAHRSupdate(gx*self.grad2rad, gy*self.grad2rad, gz*self.grad2rad, ax, ay, az, mx, my, mz)
        q = [AHRS.q0,AHRS.q1,AHRS.q2,AHRS.q3];
        #print q;
        return q;

    def quaternionToEuler(self, qt1, qt2, qt3, qt4):
        global phi; global theta; global psi;
        #R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
        r11 = 2*(qt1*qt1)-1+2*(qt2*qt2);

        #R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
        r21 = 2*(qt2*qt3-qt1*qt4);

        #R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
        r31  = 2*(qt2*qt4+qt1*qt3);

        #R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
        r32  = 2*(qt3*qt4-qt1*qt2);

        #R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
        r33 = 2*(qt1*qt1)-1+2*(qt4*qt4);

        #phi = atan2(R(3,2,:), R(3,3,:) );
        phi = numpy.math.atan2(r32, r33 ); # phi = z
        #theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
        theta = -numpy.math.atan(r31 / numpy.sqrt(1-r31*r31) ); # theta = y
        #psi = atan2(R(2,1,:), R(1,1,:) );
        psi = numpy.math.atan2(r21, r11); # psi = x
        return (phi, theta, psi)
