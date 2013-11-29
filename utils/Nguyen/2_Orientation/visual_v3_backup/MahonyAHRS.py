#//=====================================================================================================
#// MahonyAHRS.c
#//=====================================================================================================
#//
#// Madgwick's implementation of Mayhony's AHRS algorithm.
#// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
#//
#// Date         Author          Notes
#// 29/09/2011   SOH Madgwick    Initial release
#// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
#//
#//=====================================================================================================

#//---------------------------------------------------------------------------------------------------
#// Header files
import numpy
from ctypes import *

#//---------------------------------------------------------------------------------------------------
#// Definitions

sampleFreq = 50.0          #// sample frequency in Hz
twoKpDef = (2.0 * 0.5)   #// 2 * proportional gain
twoKiDef = (2.0 * 0.0)   #// 2 * integral gain

#//---------------------------------------------------------------------------------------------------
#// Variable definitions

twoKp = twoKpDef;                                            #// 2 * proportional gain (Kp)
twoKi = twoKiDef;                                            #// 2 * integral gain (Ki)
q0 = 1.0; q1 = 0.0; q2 = 0.0; q3 = 0.0;                  #// quaternion of sensor frame relative to auxiliary frame
integralFBx = 0.0;  integralFBy = 0.0; integralFBz = 0.0; #// integral error terms scaled by Ki

#//====================================================================================================
#// Functions

#//---------------------------------------------------------------------------------------------------
#// AHRS algorithm update

def MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz):
    global q0; global q1; global q2; global q3;

    recipNorm = 0.0;
    q0q0 = 0.0; q0q1 = 0.0; q0q2 = 0.0; q0q3 = 0.0; q1q1 = 0.0; q1q2 = 0.0; q1q3 = 0.0; q2q2 = 0.0; q2q3 = 0.0; q3q3 = 0.0;
    hx = 0.0; hy = 0.0; bx = 0.0; bz = 0.0;
    halfvx = 0.0; halfvy = 0.0; halfvz = 0.0; halfwx = 0.0; halfwy = 0.0; halfwz = 0.0;
    halfex = 0.0; halfey = 0.0; halfez = 0.0;
    qa = 0.0; qb = 0.0; qc = 0.0;

    #// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0) and (my == 0.0) and (mz == 0.0)):
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return (q0, q1, q2, q3);

    #// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

        #// Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        #// Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        #// Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        #// Reference direction of Earth's magnetic field
        hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = numpy.sqrt(hx * hx + hy * hy);
        bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2));

        #// Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5 + q3q3;
        halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2);

        #// Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        #// Compute and apply integral feedback if enabled
        if(twoKi > 0.0):
            integralFBx += twoKi * halfex * (1.0 / sampleFreq);    #// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0 / sampleFreq);
            integralFBz += twoKi * halfez * (1.0 / sampleFreq);
            gx += integralFBx;  #// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        else:
            integralFBx = 0.0; #// prevent integral windup
            integralFBy = 0.0;
            integralFBz = 0.0;

        #// Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;

    #// Integrate rate of change of quaternion
    gx *= (0.5 * (1.0 / sampleFreq));     #// pre-multiply common factors
    gy *= (0.5 * (1.0 / sampleFreq));
    gz *= (0.5 * (1.0 / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    #// Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    #return
    return (q0, q1, q2, q3)


#//---------------------------------------------------------------------------------------------------
#// IMU algorithm update

def MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az):
    global q0; global q1; global q2; global q3;

    recipNorm = 0.0;
    halfvx = 0.0; halfvy = 0.0; halfvz = 0.0;
    halfex = 0.0; halfey = 0.0; halfez = 0.0;
    qa = 0.0; qb = 0.0; qc = 0.0;

    #// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(not((ax == 0.0) and (ay == 0.0) and (az == 0.0))):

        #// Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        #// Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5 + q3 * q3;

        #// Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        #// Compute and apply integral feedback if enabled
        if(twoKi > 0.0):
            integralFBx += twoKi * halfex * (1.0 / sampleFreq);    #// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0 / sampleFreq);
            integralFBz += twoKi * halfez * (1.0 / sampleFreq);
            gx += integralFBx;  #// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        else:
            integralFBx = 0.0; #// prevent integral windup
            integralFBy = 0.0;
            integralFBz = 0.0;

        #// Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;

    #// Integrate rate of change of quaternion
    gx *= (0.5 * (1.0 / sampleFreq));     #// pre-multiply common factors
    gy *= (0.5 * (1.0 / sampleFreq));
    gz *= (0.5 * (1.0 / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    #// Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

#---------------------------------------------------------------------------------------------------
# Fast inverse square-root
# See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

def invSqrt(x):
    halfx = 0.5 * x;
    y = x;
    i = cast(pointer(c_float(y)), POINTER(c_long)).contents.value; #*(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = cast(pointer(c_long(i)), POINTER(c_float)).contents.value; #*(float*)&i;
    y = y * (1.5 - (halfx * y * y));
    return y;


def getMahAttitude():
  global q0; global q1; global q2; global q3;

  y1 = 2 * q2 * q3 + 2 * q0 * q1;
  x1 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  z2 = -2 * q1 * q3 + 2 * q0 * q2;
  y3 = 2 * q1 * q2 + 2 * q0 * q3;
  x3 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;

  roll = numpy.math.atan2(y1, x1); #roll (phi)
  pitch = numpy.math.asin(z2); #pitch (theta)
  yaw = numpy.math.atan2(y3, x3); #yaw (psi)
  return [roll, pitch, yaw]

#//====================================================================================================
#// END OF CODE
#//====================================================================================================
