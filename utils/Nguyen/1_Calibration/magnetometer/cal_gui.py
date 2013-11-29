"""
cal_gui.py - Calibration GUI for FreeIMU boards
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

#-------------------------------------------------------------------------------
# Name:        cal_gui
# Purpose: This module is responsible for calibration UI
#          This module is almost inherited from Varesano code with some slightly revision to work with our specific board,
#           together with some new functions such as: new connecting method, Sample method (all, x_down, y_down, z_down), Cal Plot to plot data without sampling
#
# Revisor:      nguyen
#
# Created:     02/01/2013
#-------------------------------------------------------------------------------

import sys, os
from PyQt4.QtGui import QApplication, QDialog, QMainWindow, QCursor, QFileDialog
from ui_freeimu_cal import Ui_FreeIMUCal
from PyQt4.QtCore import Qt,QObject, pyqtSlot, QThread, QSettings, SIGNAL
import numpy as np
import serial, time
from struct import unpack, pack
from binascii import unhexlify
from subprocess import call
import pyqtgraph.opengl as gl
import cal_lib, numpy

#file name
acc_file_name = "data\\acc.txt"
magn_file_name = "data\\magn.txt"
magn_full_data_file_name = "data\\magn.txt"
calibration_h_file_name = "calibration.h"

acc_range = 25000
magn_range = 1000

class FreeIMUCal(QMainWindow, Ui_FreeIMUCal):
  def __init__(self):
    QMainWindow.__init__(self)

    # Set up the user interface from Designer.
    self.setupUi(self)

    # load user settings
    self.settings = QSettings("FreeIMU Calibration Application", "Fabio Varesano")
    # restore previous serial port used
    self.serialPortEdit.setText(self.settings.value("calgui/serialPortEdit", "").toString())

    #when user changes sample type, update sample_type value
    self.cbSampleType.setCurrentIndex(0)
    self.sample_type = "all"
    self.connect(self.cbSampleType, SIGNAL("currentIndexChanged(QString)"), self.HandleSampleType)
    #self.sample_type = ""

    # when user hits enter, we generate the clicked signal to the button so that connection starts
    self.connect(self.serialPortEdit, SIGNAL("returnPressed()"), self.connectButton, SIGNAL("clicked()"))

    # Connect up the buttons to their functions
    self.connectButton.clicked.connect(self.serial_connect)
    self.samplingToggleButton.clicked.connect(self.sampling_start)

    #calibrate from file without sampling
    self.online_sample = 0
    #plot button
    self.plotButton.clicked.connect(self.plotDataFromFile)
    self.magn3D_cal_sp = None

    self.set_status("Disconnected")

    # data storages
    self.acc_data = [[], [], []]
    self.magn_data = [[], [], []]

    # setup graphs
    self.magnXY.setXRange(-magn_range, magn_range)
    self.magnXY.setYRange(-magn_range, magn_range)
    self.magnYZ.setXRange(-magn_range, magn_range)
    self.magnYZ.setYRange(-magn_range, magn_range)
    self.magnZX.setXRange(-magn_range, magn_range)
    self.magnZX.setYRange(-magn_range, magn_range)

    self.magnXY.setAspectLocked()
    self.magnYZ.setAspectLocked()
    self.magnZX.setAspectLocked()

    self.magnXY_cal.setXRange(-1.5, 1.5)
    self.magnXY_cal.setYRange(-1.5, 1.5)
    self.magnYZ_cal.setXRange(-1.5, 1.5)
    self.magnYZ_cal.setYRange(-1.5, 1.5)
    self.magnZX_cal.setXRange(-1.5, 1.5)
    self.magnZX_cal.setYRange(-1.5, 1.5)

    self.magnXY_cal.setAspectLocked()
    self.magnYZ_cal.setAspectLocked()
    self.magnZX_cal.setAspectLocked()

    self.magn3D.opts['distance'] = 2000
    self.magn3D.show()

    mx = gl.GLAxisItem()
    mx.setSize(x=1000, y=1000, z=1000)
    self.magn3D.addItem(mx)

    self.magn3D_sp = gl.GLScatterPlotItem()
    self.magn3D.addItem(self.magn3D_sp)

    # axis for the cal 3D graph
    g_m = gl.GLAxisItem()
    g_m.setSize(x=1000, y=1000, z=1000)
    self.magn3D_cal.addItem(g_m)

  def HandleSampleType(self):
    self.sample_type = str(self.cbSampleType.currentText())

  def set_status(self, status):
    self.statusbar.showMessage(self.tr(status))

  def serial_connect(self):
    self.serial_port = str(self.serialPortEdit.text())
    # save serial value to user settings
    self.settings.setValue("calgui/serialPortEdit", self.serial_port)

    self.connectButton.setEnabled(False)
    # waiting mouse cursor
    QApplication.setOverrideCursor(QCursor(Qt.WaitCursor))
    self.set_status("Connecting to " + self.serial_port + " ...")

    # TODO: serial port field input validation!

    try:
      self.ser = serial.Serial(
        port= self.serial_port,
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
      )

      if self.ser.isOpen():
        print "Arduino serial port opened correctly"
        self.set_status("Connection Successfull. Waiting for Arduino reset...")

        # wait for arduino reset on serial open
        time.sleep(3)

        self.connectButton.setText("Disconnect")
        self.connectButton.clicked.connect(self.serial_disconnect)
        self.serialPortEdit.setEnabled(False)
        self.serialProtocol.setEnabled(False)

        self.samplingToggleButton.setEnabled(True)

        self.clearCalibrationEEPROMButton.setEnabled(True)
        self.clearCalibrationEEPROMButton.clicked.connect(self.clear_calibration_eeprom)

    except serial.serialutil.SerialException, e:
      self.connectButton.setEnabled(True)
      self.set_status("Impossible to connect: " + str(e))

    # restore mouse cursor
    QApplication.restoreOverrideCursor()
    self.connectButton.setEnabled(True)

  def serial_disconnect(self):
    print "Disconnecting from " + self.serial_port
    self.ser.write("reset\r\n")
    time.sleep(1)
    self.ser.flushInput()
    self.ser.close()
    self.set_status("Disconnected")
    self.serialPortEdit.setEnabled(True)
    self.serialProtocol.setEnabled(True)

    self.connectButton.setText("Connect")
    self.connectButton.clicked.disconnect(self.serial_disconnect)
    self.connectButton.clicked.connect(self.serial_connect)

    self.samplingToggleButton.setEnabled(False)

    self.clearCalibrationEEPROMButton.setEnabled(False)
    self.clearCalibrationEEPROMButton.clicked.disconnect(self.clear_calibration_eeprom)


  def sampling_start(self):
    self.online_sample = 1
    self.serWorker = SerialWorker(ser = self.ser, sample_type = self.sample_type)
    self.connect(self.serWorker, SIGNAL("new_data(PyQt_PyObject)"), self.newData)
    self.serWorker.start()
    print "Starting SerialWorker"
    self.samplingToggleButton.setText("Stop Sampling")

    self.samplingToggleButton.clicked.disconnect(self.sampling_start)
    self.samplingToggleButton.clicked.connect(self.sampling_end)

  def sampling_end(self):
    self.serWorker.exiting = True
    self.serWorker.quit()
    self.serWorker.wait()
    #reset the sensors
    self.ser.write("reset\r\n")
    time.sleep(1)
    self.ser.flushInput()

    self.samplingToggleButton.setText("Start Sampling")
    self.samplingToggleButton.clicked.disconnect(self.sampling_end)
    self.samplingToggleButton.clicked.connect(self.sampling_start)

    self.calibrateButton.setEnabled(True)
    self.calAlgorithmComboBox.setEnabled(True)
    self.calibrateButton.clicked.connect(self.calibrate)


  #plot raw data and calibrated data
  def plotDataFromFile(self):
    #check sample type to load correct data file
    if self.sample_type == "all": #all
        magn_file_name = "data\\magn.txt"
    elif self.sample_type == "x_down": #x down
        magn_file_name = "data\\magn_xdown.txt"
    elif self.sample_type == "y_down":
        magn_file_name = "data\\magn_ydown.txt"
    elif self.sample_type == "z_down":
        magn_file_name = "data\\magn_zdown.txt"
    #read data from files
    self.magn_data = cal_lib.read_from_file(magn_file_name)
    #plot raw data
    self.magnXY.plot(x = self.magn_data[0], y = self.magn_data[1], clear = True, pen='r')
    self.magnYZ.plot(x = self.magn_data[1], y = self.magn_data[2], clear = True, pen='g')
    self.magnZX.plot(x = self.magn_data[2], y = self.magn_data[0], clear = True, pen='b')

    magn_pos = numpy.array([self.magn_data[0],self.magn_data[1],self.magn_data[2]]).transpose()
    self.magn3D_sp.setData(pos=magn_pos, color = (1, 1, 1, 1), size=2)
    #calibrate and plot calibrated data
    self.calibrate()


  #calibrate and plot calibrated data
  def calibrate(self):
    # read file and run calibration algorithm
    (self.acc_offset, self.acc_scale) = cal_lib.calibrate_from_file(acc_file_name, "acc")
    (self.magn_offset, self.magn_scale, self.magn_misalignment) = cal_lib.calibrate_from_file(magn_full_data_file_name, "mag")
    #write calibration params to file
    scale_factor_matrix = [[self.magn_scale[0], 0, 0],
                           [0, self.magn_scale[1], 0],
                           [0, 0, self.magn_scale[2]]]
    cal_lib.write_cal_params_to_file("calibration_params.txt", self.magn_misalignment, scale_factor_matrix, self.magn_offset)

    # map floats into integers
    self.acc_offset = map(int, self.acc_offset)
    self.magn_offset = map(int, self.magn_offset)

    # show calibrated tab
    self.tabWidget.setCurrentIndex(1)

    #populate magn calibration output on gui
    self.calRes_magn_OSx.setText(str(self.magn_offset[0]))
    self.calRes_magn_OSy.setText(str(self.magn_offset[1]))
    self.calRes_magn_OSz.setText(str(self.magn_offset[2]))

    self.calRes_magn_SCx.setText(str(self.magn_scale[0]))
    self.calRes_magn_SCy.setText(str(self.magn_scale[1]))
    self.calRes_magn_SCz.setText(str(self.magn_scale[2]))

    '''
    # compute calibrated data
    #check sample type to load correct data file
    if self.sample_type == "all": #all
        magn_file_name = "data\\magn.txt"
    elif self.sample_type == "x_down": #x down
        magn_file_name = "data\\magn_xdown.txt"
    elif self.sample_type == "y_down":
        magn_file_name = "data\\magn_ydown.txt"
    elif self.sample_type == "z_down":
        magn_file_name = "data\\magn_zdown.txt"
    #read data from files
    self.magn_data = cal_lib.read_from_file(magn_file_name)
    '''

    self.magn_cal_data = cal_lib.compute_full_calibrate_magnetometer_data(self.magn_data, self.magn_offset,self.magn_scale, self.magn_misalignment)
    #save calibrated data to file
    if self.sample_type == "all": #all
        cal_magn_file_name = "data\\calibrated_magn.txt"
    elif self.sample_type == "x_down": #x down
        cal_magn_file_name = "data\\calibrated_magn_xdown.txt"
    elif self.sample_type == "y_down":
        cal_magn_file_name = "data\\calibrated_magn_ydown.txt"
    elif self.sample_type == "z_down":
        cal_magn_file_name = "data\\calibrated_magn_zdown.txt"
    cal_lib.write_to_file(cal_magn_file_name, self.magn_cal_data)

    #clear existing plot data
    self.clear_plot_data()

    self.magnXY_cal.plot(x = self.magn_cal_data[0], y = self.magn_cal_data[1], clear = True, pen='r')
    self.magnYZ_cal.plot(x = self.magn_cal_data[1], y = self.magn_cal_data[2], clear = True, pen='g')
    self.magnZX_cal.plot(x = self.magn_cal_data[2], y = self.magn_cal_data[0], clear = True, pen='b')

    # populate 3D graphs with calibrated data
    magn3D_cal_data = np.array(self.magn_cal_data).transpose()

    self.magn3D_cal_sp = gl.GLScatterPlotItem(pos=magn3D_cal_data, color = (1, 1, 1, 1), size=2)
    self.magn3D_cal.addItem(self.magn3D_cal_sp)

    #enable calibration buttons to activate calibration storing functions
    self.saveCalibrationHeaderButton.setEnabled(True)
    self.saveCalibrationHeaderButton.clicked.connect(self.save_calibration_header)

    self.saveCalibrationEEPROMButton.setEnabled(True)
    self.saveCalibrationEEPROMButton.clicked.connect(self.save_calibration_eeprom)

  def clear_plot_data(self):
    empty_data = [[],[],[]]
    self.magnXY_cal.plot(x = empty_data[0], y = empty_data[1], clear = True, pen='r')
    self.magnYZ_cal.plot(x = empty_data[1], y = empty_data[2], clear = True, pen='g')
    self.magnZX_cal.plot(x = empty_data[2], y = empty_data[0], clear = True, pen='b')
    #3D
    empty_3D_data = np.array(empty_data).transpose()
    if(self.magn3D_cal_sp != None):
        self.magn3D_cal.removeItem(self.magn3D_cal_sp)
    sp = gl.GLScatterPlotItem(pos=empty_3D_data, color = (1, 1, 1, 1), size=2)
    self.magn3D_cal.addItem(sp)

  '''
  #Nguyen
  def plotNewData(self):
    #compute calibrate params
    (self.magn_offset, self.magn_scale, self.magn_misalignment) = cal_lib.calibrate_from_file(magn_full_data_file_name, "mag")
    # compute calibrated data
    if(self.online_sample == 0):
        #check sample type to load correct data file
        if self.sample_type == "all": #all
            magn_file_name = "data\\magn.txt"
        elif self.sample_type == "x_down": #x down
            magn_file_name = "data\\magn_xdown.txt"
        elif self.sample_type == "y_down":
            magn_file_name = "data\\magn_ydown.txt"
        elif self.sample_type == "z_down":
            magn_file_name = "data\\magn_zdown.txt"
        #read data from files
        self.acc_data = cal_lib.read_from_file(acc_file_name)
        self.magn_data = cal_lib.read_from_file(magn_file_name)
    #calculate calibrated data
    self.magn_cal_data = cal_lib.compute_full_calibrate_magnetometer_data(self.magn_data, self.magn_offset,self.magn_scale, self.magn_misalignment)
    #save calibrated data to file
    if self.sample_type == "all": #all
        cal_magn_file_name = "data\\calibrated_magn.txt"
    elif self.sample_type == "x_down": #x down
        cal_magn_file_name = "data\\calibrated_magn_xdown.txt"
    elif self.sample_type == "y_down":
        cal_magn_file_name = "data\\calibrated_magn_ydown.txt"
    elif self.sample_type == "z_down":
        cal_magn_file_name = "data\\calibrated_magn_zdown.txt"
    cal_lib.write_to_file(cal_magn_file_name, self.magn_cal_data)
    # populate 2D graphs with calibrated data
    self.magnXY_cal.plot(x = self.magn_cal_data[0], y = self.magn_cal_data[1], clear = True, pen='r')
    self.magnYZ_cal.plot(x = self.magn_cal_data[1], y = self.magn_cal_data[2], clear = True, pen='g')
    self.magnZX_cal.plot(x = self.magn_cal_data[2], y = self.magn_cal_data[0], clear = True, pen='b')

    # populate 3D graphs with calibrated data
    magn3D_cal_data = np.array(self.magn_cal_data).transpose()

    self.magn3D_cal_sp = gl.GLScatterPlotItem(pos=magn3D_cal_data, color = (1, 1, 1, 1), size=2)
    self.magn3D_cal.addItem(self.magn3D_cal_sp)

  #Nguyen
  def plotData(self):
    self.clear_plot_data()
    self.plotNewData()
  '''

  def save_calibration_header(self):
    text = """
    /**
     * FreeIMU calibration header. Automatically generated by FreeIMU_GUI.
     * Do not edit manually unless you know what you are doing.
    */


    #define CALIBRATION_H

    const int acc_off_x = %d;
    const int acc_off_y = %d;
    const int acc_off_z = %d;
    const float acc_scale_x = %f;
    const float acc_scale_y = %f;
    const float acc_scale_z = %f;

    const int magn_off_x = %d;
    const int magn_off_y = %d;
    const int magn_off_z = %d;
    const float magn_scale_x = %f;
    const float magn_scale_y = %f;
    const float magn_scale_z = %f;

    const float magn_misalign_Mxx = %f;
    const float magn_misalign_Mxy = %f;
    const float magn_misalign_Mxz = %f;

    const float magn_misalign_Myx = %f;
    const float magn_misalign_Myy = %f;
    const float magn_misalign_Myz = %f;

    const float magn_misalign_Mzx = %f;
    const float magn_misalign_Mzy = %f;
    const float magn_misalign_Mzz = %f;
    """
    calibration_h_text = text % (self.acc_offset[0], self.acc_offset[1], self.acc_offset[2], self.acc_scale[0], self.acc_scale[1], self.acc_scale[2], self.magn_offset[0], self.magn_offset[1], self.magn_offset[2], self.magn_scale[0], self.magn_scale[1], self.magn_scale[2], self.magn_misalignment[0][0], self.magn_misalignment[0][1], self.magn_misalignment[0][2], self.magn_misalignment[1][0],self.magn_misalignment[1][1],self.magn_misalignment[1][2],self.magn_misalignment[2][0],self.magn_misalignment[2][1],self.magn_misalignment[2][2])

    calibration_h_folder = QFileDialog.getExistingDirectory(self, "Select the Folder to which save the calibration.h file")
    calibration_h_file = open(os.path.join(str(calibration_h_folder), calibration_h_file_name), "w")
    calibration_h_file.write(calibration_h_text)
    calibration_h_file.close()

    self.set_status("Calibration saved to: " + str(calibration_h_folder) + calibration_h_file_name + " .\nRecompile and upload the program using the FreeIMU library to your microcontroller.")

  def save_calibration_eeprom(self):
    self.ser.write("c")
    # pack data into a string
    offsets = pack('<hhhhhh', self.acc_offset[0], self.acc_offset[1], self.acc_offset[2], self.magn_offset[0], self.magn_offset[1], self.magn_offset[2])
    scales = pack('<ffffff', self.acc_scale[0], self.acc_scale[1], self.acc_scale[2], self.magn_scale[0], self.magn_scale[1], self.magn_scale[2])
    # transmit to microcontroller
    self.ser.write(offsets)
    self.ser.write(scales)
    self.set_status("Calibration saved to microcontroller EEPROM.")
    # debug written values to console
    print "Calibration values read back from EEPROM:"
    self.ser.write("C")
    for i in range(4):
      print self.ser.readline()


  def clear_calibration_eeprom(self):
    self.ser.write("x")
    # no feedback expected. we assume success.
    self.set_status("Calibration cleared from microcontroller EEPROM.")


  def newData(self, reading):

    # only display last reading in burst
    self.magn_data[0].append(reading[6])
    self.magn_data[1].append(reading[7])
    self.magn_data[2].append(reading[8])

    self.magnXY.plot(x = self.magn_data[0], y = self.magn_data[1], clear = True, pen='r')
    self.magnYZ.plot(x = self.magn_data[1], y = self.magn_data[2], clear = True, pen='g')
    self.magnZX.plot(x = self.magn_data[2], y = self.magn_data[0], clear = True, pen='b')

    magn_pos = numpy.array([self.magn_data[0],self.magn_data[1],self.magn_data[2]]).transpose()
    self.magn3D_sp.setData(pos=magn_pos, color = (1, 1, 1, 1), size=2)


class SerialWorker(QThread):
  def __init__(self, parent = None, ser = None, sample_type = None):
    QThread.__init__(self, parent)
    self.exiting = False
    self.ser = ser
    self.sample_type = sample_type

  def run(self):
    print "sampling start.."
    print ("sample type: " + str(self.sample_type))
    if self.sample_type == "all": #all
        magn_file_name = "data\\magn.txt"
        magn_full_data_file_name = magn_file_name
    elif self.sample_type == "x_down": #x down
        magn_file_name = "data\\magn_xdown.txt"
    elif self.sample_type == "y_down":
        magn_file_name = "data\\magn_ydown.txt"
    elif self.sample_type == "z_down":
        magn_file_name = "data\\magn_zdown.txt"

    self.acc_file = open(acc_file_name, 'w')
    self.magn_file = open(magn_file_name, 'w')
    count = 2  #100
    in_values = 9
    reading = [0.0 for i in range(in_values)]
    buff = []

    #------------------start
    cmd = "sraw "
    self.ser.write("reset\r\n")
    time.sleep(1)
    self.ser.write(cmd + str(10) + "\r\n")
    time.sleep(1)
    self.ser.flushInput()
    #----------------- end

    while not self.exiting:
      #---------------------- start
      for j in range(count):
        line = self.ser.readline()
        line = line [: len(line) - 2] #eliminate \r\n at the end
        words = line.split()
        i = 1
        for word in words:
            if i > 1:
                reading[i-2] = int(word) # reading: contains 9 values from gx gy gz ax ay az mx my mz
            i = i + 1
        # prepare readings to store on file
        acc_readings_line = "%d %d %d\r\n" % (reading[3], reading[4], reading[5])
        self.acc_file.write(acc_readings_line)
        magn_readings_line = "%d %d %d\r\n" % (reading[6], reading[7], reading[8])
        self.magn_file.write(magn_readings_line)
      self.emit(SIGNAL("new_data(PyQt_PyObject)"), reading)
      buff = []
      print ".",

    # closing acc and magn files
    self.acc_file.close()
    self.magn_file.close()
    return

  def __del__(self):
    self.exiting = True
    self.wait()
    print "SerialWorker exits.."


app = QApplication(sys.argv)
window = FreeIMUCal()

window.show()
sys.exit(app.exec_())