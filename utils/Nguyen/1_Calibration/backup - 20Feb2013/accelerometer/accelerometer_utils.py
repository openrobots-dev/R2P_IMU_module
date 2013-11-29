#-------------------------------------------------------------------------------
# Name:        accelerometer_utils
# Purpose: Contains several utility functions for calibration, sharing among several modules
#
# Author:      nguyen
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



#####################################################   FROM PAPARAZZI - ONLY REFERENCES PURPOSE  #####################################################################
#
# select only non-noisy data
#
def filter_meas(meas, window_size, noise_threshold):
    filtered_meas = []
    filtered_idx = []
    for i in range(window_size, len(meas)-window_size):
        noise = meas[i-window_size:i+window_size,:].std(axis=0) #calculate standard deviation on axis 0
        if  linalg.norm(noise) < noise_threshold:
            filtered_meas.append(meas[i,:])
            filtered_idx.append(i)
    return scipy.array(filtered_meas), filtered_idx


#
# initial boundary based calibration
#
def get_min_max_guess(meas, scale):
    max_meas = meas[:,:].max(axis=0)
    min_meas = meas[:,:].min(axis=0)
    n = (max_meas + min_meas) / 2
    sf = 2*scale/(max_meas - min_meas)
    return scipy.array([n[0], n[1], n[2], sf[0], sf[1], sf[2]])


#
# scale the set of measurements
#
def scale_measurements(meas, p):
    l_comp = [];
    l_norm = [];
    for m in meas[:,]:
        sm = (m - p[0:3])*p[3:6]
        l_comp.append(sm)
        l_norm.append(linalg.norm(sm))
    return scipy.array(l_comp), scipy.array(l_norm)


#
# print xml for airframe file
#
def print_xml(p, sensor, res):
    print("")
    print("<define name=\""+sensor+"_X_NEUTRAL\" value=\""+str(int(round(p[0])))+"\"/>")
    print("<define name=\""+sensor+"_Y_NEUTRAL\" value=\""+str(int(round(p[1])))+"\"/>")
    print("<define name=\""+sensor+"_Z_NEUTRAL\" value=\""+str(int(round(p[2])))+"\"/>")
    print("<define name=\""+sensor+"_X_SENS\" value=\""+str(p[3]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Y_SENS\" value=\""+str(p[4]*2**res)+"\" integer=\"16\"/>")
    print("<define name=\""+sensor+"_Z_SENS\" value=\""+str(p[5]*2**res)+"\" integer=\"16\"/>")

#plot low pass filter
def plot_lowpass_filter(block, measurements, smooth_meas):
    subplot(3, 1, 1)
    plot(measurements[0:len(measurements)], label = "raw")
    plot(smooth_meas[0:len(smooth_meas)])
    #plot(measurements[:, 2])
    xlabel('time (s)')
    ylabel('ADC')
    title('Raw sensors')
     # if we want to have another plot we only draw the figure (non-blocking)
    # also in matplotlib before 1.0.0 there is only one call to show possible
    if block:
        show()
    else:
        draw()

#
# plot calibration results
#
def plot_results(block, measurements, flt_idx, flt_meas, cp0, np0, cp1, np1, sensor_ref):
    subplot(3, 1, 1)
    plot(measurements[:, 0])
    plot(measurements[:, 1])
    plot(measurements[:, 2])
    plot(flt_idx, flt_meas[:, 0], 'ro')
    plot(flt_idx, flt_meas[:, 1], 'ro')
    plot(flt_idx, flt_meas[:, 2], 'ro')
    xlabel('time (s)')
    ylabel('ADC')
    title('Raw sensors')

    subplot(3, 2, 3)
    plot(cp0[:, 0]);
    plot(cp0[:, 1]);
    plot(cp0[:, 2]);
    plot(-sensor_ref*scipy.ones(len(flt_meas)));
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3, 2, 4)
    plot(np0);
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3, 2, 5)
    plot(cp1[:, 0]);
    plot(cp1[:, 1]);
    plot(cp1[:, 2]);
    plot(-sensor_ref*scipy.ones(len(flt_meas)));
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    subplot(3, 2, 6)
    plot(np1);
    plot(sensor_ref*scipy.ones(len(flt_meas)));

    # if we want to have another plot we only draw the figure (non-blocking)
    # also in matplotlib before 1.0.0 there is only one call to show possible
    if block:
        show()
    else:
        draw()

#
# plot mag measurements in 3D
#
def plot_mag_3d(measured, calibrated, p):
    # set up points for sphere and ellipsoid wireframes
    u=r_[0:2*pi:20j]
    v=r_[0:pi:20j]
    wx=outer(cos(u),sin(v))
    wy=outer(sin(u),sin(v))
    wz=outer(ones(size(u)),cos(v))
    ex=p[0]*ones(size(u)) + outer(cos(u),sin(v))/p[3]
    ey=p[1]*ones(size(u)) + outer(sin(u),sin(v))/p[4]
    ez=p[2]*ones(size(u)) + outer(ones(size(u)),cos(v))/p[5]

    # measurements
    mx = measured[:, 0]
    my = measured[:, 1]
    mz = measured[:, 2]
    m_max = amax(abs(measured))

    # calibrated values
    cx = calibrated[:, 0]
    cy = calibrated[:, 1]
    cz = calibrated[:, 2]

    # axes size
    left = 0.02
    bottom = 0.05
    width = 0.46
    height = 0.9
    rect_l = [left, bottom, width, height]
    rect_r = [left/2+0.5, bottom, width, height]

    fig = figure(figsize=figaspect(0.5))
    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_l)
    else:
        ax = fig.add_subplot(1, 2, 1, position=rect_l, projection='3d')
    # plot measurements
    ax.scatter(mx, my, mz)
    hold(True)
    # plot line from center to ellipsoid center
    ax.plot([0.0, p[0]], [0.0, p[1]], [0.0, p[2]], color='black', marker='+')
    # plot ellipsoid
    ax.plot_wireframe(ex, ey, ez, color='grey', alpha=0.5)

    ax.set_title('MAG raw with fitted ellipsoid and center offset')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d(-m_max, m_max)
    ax.set_ylim3d(-m_max, m_max)
    ax.set_zlim3d(-m_max, m_max)

    if matplotlib.__version__.startswith('0'):
        ax = Axes3D(fig, rect=rect_r)
    else:
        ax = fig.add_subplot(1, 2, 2, position=rect_r, projection='3d')
    ax.plot_wireframe(wx, wy, wz, color='grey', alpha=0.5)
    hold(True)
    ax.scatter(cx, cy, cz)

    ax.set_title('MAG calibrated on unit sphere')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)
    show()

#
# read a turntable log
# return an array which first column is turnatble and next 3 are gyro
#
def read_turntable_log(ac_id, tt_id, filename, _min, _max):
    f = open(filename, 'r')
    pattern_g = re.compile("(\S+) " + str(ac_id) + " IMU_GYRO_RAW (\S+) (\S+) (\S+)")
    pattern_t = re.compile("(\S+) " + str(tt_id) + " IMU_TURNTABLE (\S+)")
    last_tt = None
    list_tt = []
    while True:
        line = f.readline().strip()
        if line == '':
            break
        m=re.match(pattern_t, line)
        if m:
            last_tt = float(m.group(2))
        m=re.match(pattern_g, line) #if m and last_tt and last_tt > _min and last_tt < _max:
        if m and last_tt and last_tt > _min and last_tt < _max:
            list_tt.append([last_tt, float(m.group(2)), float(m.group(3)), float(m.group(4))])
    return scipy.array(list_tt)

