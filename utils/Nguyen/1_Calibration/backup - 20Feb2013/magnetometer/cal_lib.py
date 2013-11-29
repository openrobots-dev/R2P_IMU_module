"""
cal_lib.py - Ellipsoid into Sphere calibration library based upon numpy and linalg
Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

import numpy
from numpy import linalg

#VARIABLES
x_file = "data\\magn_xdown.txt"
y_file = "data\\magn_ydown.txt"
z_file = "data\\magn_zdown.txt"

OSx = 0 #offset x
OSy = 0 #offset y
OSz = 0 #offset z
#scale factor
SCx = 0.0
SCy = 0.0
SCz = 0.0
#misalignment
Mxy = 0.0
Mxz = 0.0
Myx = 0.0
Myz = 0.0
Mzx = 0.0
Mzy = 0.0

def read_from_file(file_name):
    data = [[], [], []]
    f = open(file_name, 'r')
    for line in f:
        reading = line.split()
        data[0].append(int(reading[0]))
        data[1].append(int(reading[1]))
        data[2].append(int(reading[2]))
    f.close()
    return data

def write_to_file(filename, data):
    f = open(filename, 'w')
    for i in range(len(data[0])):
        line = str(data[0][i]) + " " + str(data[1][i]) + " " + str(data[2][i]) + "\n"
        f.write(line)
    f.close()

def compute_full_calibrate_magnetometer_data(data, offsets, scale, misalignment):
  output = [[], [], []]
  scale_matrix = [[1/scale[0], 0, 0],
                  [0, 1/scale[1], 0],
                  [0, 0, 1/scale[2]]]
  dot_matrix = numpy.dot(misalignment,scale_matrix)
  for i in range(len(data[0])):
    raw = [(data[0][i] - offsets[0]), (data[1][i] - offsets[1]), (data[2][i] - offsets[2])]
    raw = numpy.transpose(raw)
    calibrated_data = numpy.dot(dot_matrix, raw)
    output[0].append(calibrated_data[0])
    output[1].append(calibrated_data[1])
    output[2].append(calibrated_data[2])
  return output

# Full calibration which calculates misalignment errors, scale factors, offset
#Solves the equation a x = b by computing a vector x that minimizes the Euclidean 2-norm || b - a x ||^2
#numpy.linalg.lstsq(a, b, rcond=-1)
def calibrate_misalignment(x_file, y_file, z_file):
    #z down
    samples_z = open(z_file, 'r')
    Hz = []
    Wz = []
    for line in samples_z:
        reading = line.split()
        if len(reading) == 3:
            Hz.append([int(reading[0]), int(reading[1]), int(reading[2])])
            #normalize
            norm = linalg.norm([int(reading[0]), int(reading[1]), int(reading[2])])
            Wz.append([norm])

    samples_z.close()
    Hz_transpose = numpy.transpose(Hz)
    Xz = numpy.dot(numpy.dot(linalg.inv(numpy.dot(Hz_transpose,Hz)),Hz_transpose),Wz)
    #(Xz, residues, rank, shape) = linalg.lstsq(Hz,Wz)
    Rz = Xz / linalg.norm(Xz)

    #y down
    samples_y = open(y_file, 'r')
    Hy = []
    Wy = []
    for line in samples_y:
        reading = line.split()
        if len(reading) == 3:
            Hy.append([int(reading[0]), int(reading[1]), int(reading[2])])
            #normalize
            norm = linalg.norm([int(reading[0]), int(reading[1]), int(reading[2])])
            Wy.append([norm])

    samples_y.close()
    Hy_transpose = numpy.transpose(Hy)
    Xy = numpy.dot(numpy.dot(linalg.inv(numpy.dot(Hy_transpose,Hy)),Hy_transpose),Wy)
    Ry = Xy / linalg.norm(Xy)

    #x down
    samples_x = open(x_file, 'r')
    Hx = []
    Wx = []
    for line in samples_x:
        reading = line.split()
        if len(reading) == 3:
            Hx.append([int(reading[0]), int(reading[1]), int(reading[2])])
            #normalize
            norm = linalg.norm([int(reading[0]), int(reading[1]), int(reading[2])])
            Wx.append([norm])

    samples_x.close()
    Hx_transpose = numpy.transpose(Hx)
    Xx = numpy.dot(numpy.dot(linalg.inv(numpy.dot(Hx_transpose,Hx)),Hx_transpose),Wx)
    #(Xxx, residues, rank, shape) = linalg.lstsq(Hx,Wx)
    Rx = Xx / linalg.norm(Xx)
    #return misalignment matrix
    Rx = (numpy.transpose(Rx))[0]
    Ry = (numpy.transpose(Ry))[0]
    Rz = (numpy.transpose(Rz))[0]
    mis_matrix = [[Rx[0], Rx[1], Rx[2]],
                  [Ry[0], Ry[1], Ry[2]],
                  [Rz[0], Rz[1], Rz[2]]]

    return mis_matrix


def calibrate(x, y, z, sensor_type):
  H = numpy.array([x, y, z, -y**2, -z**2, numpy.ones([len(x), 1])])
  H = numpy.transpose(H)
  w = x**2

  (X, residues, rank, shape) = linalg.lstsq(H, w)

  OSx = X[0] / 2
  OSy = X[1] / (2 * X[3])
  OSz = X[2] / (2 * X[4])

  A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
  B = A / X[3]
  C = A / X[4]

  SCx = numpy.sqrt(A)
  SCy = numpy.sqrt(B)
  SCz = numpy.sqrt(C)

  # type conversion from numpy.float64 to standard python floats
  offsets = [OSx, OSy, OSz]
  scale = [SCx, SCy, SCz]

  offsets = map(numpy.asscalar, offsets)
  scale = map(numpy.asscalar, scale)

  #misalignment matrix
  if(sensor_type == "mag"):
    mis_matrix = calibrate_misalignment(x_file, y_file, z_file)
    return (offsets, scale, mis_matrix)
  else:
    return (offsets, scale)


def calibrate_from_file(file_name, sensor_type):
  samples_f = open(file_name, 'r')
  samples_x = []
  samples_y = []
  samples_z = []
  for line in samples_f:
    reading = line.split()
    if len(reading) == 3:
      samples_x.append(int(reading[0]))
      samples_y.append(int(reading[1]))
      samples_z.append(int(reading[2]))

  samples_f.close()
  return calibrate(numpy.array(samples_x), numpy.array(samples_y), numpy.array(samples_z), sensor_type)


def compute_calibrate_data(data, offsets, scale):
  output = [[], [], []]
  for i in range(len(data[0])):
    output[0].append((data[0][i] - offsets[0]) / scale[0])
    output[1].append((data[1][i] - offsets[1]) / scale[1])
    output[2].append((data[2][i] - offsets[2]) / scale[2])
  return output


if __name__ == "__main__":

  print "Calibrating from acc.txt"
  (offsets, scale) = calibrate_from_file("data\\acc.txt", "acc")
  print "Offsets:"
  print offsets
  print "Scales:"
  print scale

  print "Calibrating from magn.txt"
  (offsets, scale) = calibrate_from_file("data\\magn.txt", "mag")
  print "Offsets:"
  print offsets
  print "Scales:"
  print scale