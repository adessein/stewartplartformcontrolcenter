#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 21:00:10 2017

Integration of Matplotlib in Qt
https://matplotlib.org/examples/user_interfaces/embedding_in_qt4.html

PyQt4 doc
http://pyqt.sourceforge.net/Docs/PyQt4/

qwt


@author: arnaud
"""

from PyQt4 import QtCore, QtGui, uic
import PyQt4.Qwt5 as Qwt
from sys import argv
import sys
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import threading
import serial
from PyQt4.QtCore import QObject, pyqtSignal

class MainWindow(QtGui.QMainWindow):
  
  def __init__(self, parent=None):
    QtGui.QMainWindow.__init__(self, parent)
    self.setFixedSize(1190,780)
    self.setWindowTitle('Stewart platform control center')
    
    centralArea = QtGui.QWidget()
    lay0 = QtGui.QHBoxLayout()
    
    lay1 = QtGui.QVBoxLayout()
    
    # Layout 11
    lay11 = QtGui.QGridLayout()
    
    self.pitchCompass = Qwt.QwtCompass()
    self.pitchCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    
    self.rollCompass = Qwt.QwtCompass()
    self.rollCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    self.rollCompass.setValue(90)
    
    self.haDial = Qwt.QwtDial()
    self.haDial.setMode(Qwt.QwtDial.RotateScale)
    
    figBall = Figure(figsize=(210, 170), dpi=120)
    self.fcBall = FigureCanvas(figBall)
    
    lay11.addWidget(self.pitchCompass, 0, 0)
    lay11.addWidget(self.fcBall, 0, 1)
    lay11.addWidget(self.haDial, 1, 0)
    lay11.addWidget(self.rollCompass, 1, 1)
    
    lay11.setColumnStretch(0,1)
    lay11.setColumnStretch(1,1)
    lay11.setRowStretch(0,1)
    lay11.setRowStretch(1,1)
    
    # Layout 12
    lay12 = QtGui.QGridLayout()
    
    self.ths = {}
    for i in range(6):
      self.ths[i] = QtGui.QSlider()
      self.ths[i].setMinimum(-90)
      self.ths[i].setMaximum(90)
      self.ths[i].setPageStep(1)
      self.ths[i].setTickPosition(QtGui.QSlider.TicksBothSides)
      self.ths[i].setTickInterval(15)
      self.ths[i].setMinimumSize(38,111)
      self.ths[i].setMaximumSize(38,111)
    
    lay12.addWidget(self.ths[0],0,1)
    lay12.addWidget(self.ths[1],0,2)
    lay12.addWidget(self.ths[2],1,0)
    lay12.addWidget(self.ths[3],1,3)
    lay12.addWidget(self.ths[4],2,1)
    lay12.addWidget(self.ths[5],2,2)
    
    lay1.addLayout(lay11)
    lay1.addLayout(lay12)
    lay1.setStretchFactor(lay11,1)
    lay1.setStretchFactor(lay12,1)
    
    # Layout 2
    lay2 = QtGui.QHBoxLayout()
    
    
    # Layout 21
    lay21 = QtGui.QVBoxLayout()

    fig3d = Figure(figsize=(210, 170), dpi=120)
    self.fc3d = FigureCanvas(fig3d)
    
    figprog = Figure(figsize=(210, 170), dpi=120)
    self.fcprog = FigureCanvas(figprog)

    groupP = QtGui.QGroupBox('Platform')
    groupN = QtGui.QGroupBox('Nunchuck')
    
    groupPl = QtGui.QHBoxLayout()
    groupNl = QtGui.QHBoxLayout()
    
    groupP1 = QtGui.QFormLayout()
    groupP2 = QtGui.QFormLayout()
    groupN1 = QtGui.QFormLayout()
    groupN2 = QtGui.QFormLayout()
    
    labelP1 = QtGui.QLabel('X')
    labelP2 = QtGui.QLabel('Y')
    labelP3 = QtGui.QLabel('Z')
    labelP4 = QtGui.QLabel('Rho')
    labelP5 = QtGui.QLabel('Phi')
    labelP6 = QtGui.QLabel('Theta')
    
    labelN1 = QtGui.QLabel('X')
    labelN2 = QtGui.QLabel('Y')
    labelN3 = QtGui.QLabel('Z')
    labelN4 = QtGui.QLabel('Rho')
    labelN5 = QtGui.QLabel('Phi')
    labelN6 = QtGui.QLabel('Theta')
    
    self.lcdP = {}
    self.lcdN = {}
    for i in range(6):
      self.lcdP[i] = QtGui.QLCDNumber()
      self.lcdN[i] = QtGui.QLCDNumber()
    
    groupP1.addRow(labelP1, self.lcdP[0])
    groupP1.addRow(labelP2, self.lcdP[1])
    groupP1.addRow(labelP3, self.lcdP[2])    
    groupP2.addRow(labelP4, self.lcdP[3])
    groupP2.addRow(labelP5, self.lcdP[4])
    groupP2.addRow(labelP6, self.lcdP[5])
    
    groupN1.addRow(labelN1, self.lcdN[0])
    groupN1.addRow(labelN2, self.lcdN[1])
    groupN1.addRow(labelN3, self.lcdN[2])    
    groupN2.addRow(labelN4, self.lcdN[3])
    groupN2.addRow(labelN5, self.lcdN[4])
    groupN2.addRow(labelN6, self.lcdN[5])
    
    groupPl.addLayout(groupP1)
    groupPl.addLayout(groupP2)
    groupNl.addLayout(groupN1)
    groupNl.addLayout(groupN2)
    
    groupP.setLayout(groupPl)
    groupN.setLayout(groupNl)

    lay21.addWidget(self.fc3d)
    lay21.addWidget(self.fcprog)
    lay21.addWidget(groupP)
    lay21.addWidget(groupN)
    lay21.setStretchFactor(self.fc3d,2)
    lay21.setStretchFactor(self.fcprog,2)
    lay21.setStretchFactor(groupP,1)
    lay21.setStretchFactor(groupN,1)
  
    
    # Layout 22
    lay22 = QtGui.QVBoxLayout()
    lay221 = QtGui.QFormLayout()
    self.dsbP = QtGui.QDoubleSpinBox()    
    self.dsbI = QtGui.QDoubleSpinBox()
    self.dsbD = QtGui.QDoubleSpinBox()
    lay221.addRow("P", self.dsbP)
    lay221.addRow("I", self.dsbI)
    lay221.addRow("D", self.dsbD)
    
    self.tbTargetList = QtGui.QTableView()
    
    lay22.addLayout(lay221)
    lay22.addWidget(self.tbTargetList)
    lay22.addItem(QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding))
    
    lay2.addLayout(lay21)
    lay2.addLayout(lay22)
    lay2.setStretchFactor(lay21,1)
    lay2.setStretchFactor(lay22,1)
    
    lay3 = QtGui.QVBoxLayout()
    lay31 = QtGui.QVBoxLayout()
    lay32 = QtGui.QGridLayout()
    lay33 = QtGui.QHBoxLayout()
    lay331 = QtGui.QGridLayout()
    lay332 = QtGui.QVBoxLayout()
    
    self.pbAutoMan = QtGui.QPushButton("Automatic")
    self.pbNunGui = QtGui.QPushButton("Nunchuck")
    self.pbPlatSens = QtGui.QPushButton("Platform")
    self.pbAutoMan.setCheckable(True)
    self.pbNunGui.setCheckable(True)
    self.pbPlatSens.setCheckable(True)
    
    #pbAutoManIcon = QtGui.QIcon()
    #pbAutoManIcon.addPixmap(QtGui.QPixmap('auto.png'), QtGui.QIcon.Normal, QtGui.QIcon.On)
    #pbAutoManIcon.addPixmap(QtGui.QPixmap('manual.png'), QtGui.QIcon.Normal, QtGui.QIcon.Off)
    #pbAutoMan.setIcon(pbAutoManIcon)

    #pbAutoMan.setIconSize(QSize(pbAutoMan.size()
    
    for ts in [self.pbAutoMan, self.pbNunGui, self.pbPlatSens]:
      lay31.addWidget(ts)
      
    self.pbCenter = QtGui.QPushButton("Centre")
    self.pbCircle = QtGui.QPushButton("Circle")
    self.pbSquare = QtGui.QPushButton("Square")
    self.pbTriangle = QtGui.QPushButton("Triangle")
    
    lay32.addWidget(self.pbCenter,0,0)
    lay32.addWidget(self.pbCircle,0,1)
    lay32.addWidget(self.pbSquare,1,0)
    lay32.addWidget(self.pbTriangle,1,1)
    
    self.sls = {}
    for i in range(6):
      self.sls[i] = QtGui.QSlider()
      self.sls[i].setMinimum(-90)
      self.sls[i].setMaximum(90)
      self.sls[i].setPageStep(1)
      self.sls[i].setTickPosition(QtGui.QSlider.TicksBothSides)
      self.sls[i].setTickInterval(15)
      self.sls[i].setMinimumSize(28,145)
      self.sls[i].setMaximumSize(28,145)
      lay331.addWidget(self.sls[i],0,i)
      lab = QtGui.QLabel(str(i))
      lab.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
      lay331.addWidget(lab,1,i)
    
    lay331.setRowStretch(6,1)
    
    self.pbHome = QtGui.QPushButton("Home")
    self.pbZero = QtGui.QPushButton("0 deg")
    lay332.addWidget(self.pbHome)
    lay332.addWidget(self.pbZero)
    lay332.addItem(QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding))
    
    lay33.addLayout(lay331)
    lay33.addLayout(lay332)
    lay33.setStretchFactor(lay331,3)
    lay33.setStretchFactor(lay332,1)
    
    lay3.addLayout(lay31)
    lay3.addLayout(lay32)
    lay3.addLayout(lay33)
    lay3.setStretchFactor(lay31,2)
    lay3.setStretchFactor(lay32,1)
    lay3.setStretchFactor(lay33,3)
    
    lay0.addLayout(lay1)
    lay0.addLayout(lay2)
    lay0.addLayout(lay3)
    lay0.setStretchFactor(lay1,2)
    lay0.setStretchFactor(lay2,2)
    lay0.setStretchFactor(lay3,1)
    
    centralArea.setLayout(lay0)
    self.setCentralWidget(centralArea)
    
    self.monitor = SerialMonitor()
    self.monitor.bufferUpdated.connect(self.update)
    self.monitor.start()
    
  def update(self, msg):
    print(msg)
    print msg
    self.pbCenter.setText(msg)
      

class SerialMonitor(QObject):
  """
  From https://codereview.stackexchange.com/questions/142130/serial-port-data-plotter-in-pyqt
  """
  
  bufferUpdated = pyqtSignal(unicode)

  def __init__(self):
    super(SerialMonitor, self).__init__()
    self.running = False
    self.thread = threading.Thread(target=self.serial_monitor_thread)

  def start(self):
    print("Starting serial thread")
    self.ser = serial.open('/dev/pts/2')   # Testing
    #ser = serial.Serial('/dev/ttyACM0') # Arduino
    print("the serial port is")
    print(self.ser)
    self.running = True
    self.thread.start()

  def stop(self):
    print("Stopping serial thread")
    self.running = False
    self.ser.close()

  def serial_monitor_thread(self):
    while self.running is True:
      msg = self.ser.readline().decode('ascii')
      if msg:
        try:
          self.bufferUpdated.emit(msg)
        except ValueError:
          print('Wrong data')
      else:
        pass
      
      
if __name__ == "__main__":
    #app = QtGui.QApplication(argv)
    mainWindow = MainWindow()
    mainWindow.show()
    #sys.exit(app.exec_())
