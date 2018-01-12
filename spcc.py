#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 21:00:10 2017

Integration of Matplotlib in Qt
https://matplotlib.org/examples/user_interfaces/embedding_in_qt4.html

Refresh a plot / animated plot
http://ybenabbas.cu.cc/index.php/tutorials/10-creer-un-graphe-dynamique-avec-pyplot-et-pylab?showall=&start=1

PyQt4 doc
http://pyqt.sourceforge.net/Docs/PyQt4/

qwt

It is important to install pyserial, not just the package python-serial
pip2 install pyserial

(yaw, psi)
   (Z)----------->  X  (roll, phi)
    |
    |
    |
    | 
   \|/
    Y
   
    Y (pitch, theta)


Simulation of the seral conection

socat -d -d pty,rawer  pty,rawer
echo -e "CG 11.00000 22.00000 33.00000 1.00000 2.00000 3.00000 " > /dev/pts/1

@author: arnaud
"""

from PyQt4 import QtCore, QtGui, uic
import PyQt4.Qwt5 as Qwt
from sys import argv
import sys
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D
from matplotlib.figure import Figure
import threading
import serial
from PyQt4.QtCore import QObject, pyqtSignal, QTimer
from numpy import zeros
from math import degrees, radians, cos, sin

#serialInterface = '/dev/pts/2' # test
serialInterface = '/dev/ttyACM4' # arduino
serialPeriod = 100 #ms
serialTimeout = 0.001 #s
serialSpeed = 115200

class MainWindow(QtGui.QMainWindow):
  
  def __init__(self, parent=None):
    QtGui.QMainWindow.__init__(self, parent)
    
    self.setupGui()
    
    # Internal data from the system
    self.alpha = zeros((6,))
    self.beta = zeros((6,))
    self.ballState  = zeros((6,)) # x y vx vy ax ay
    self.platPos  = zeros((6,)) # x y z phi theta psi
    self.b = zeros((6,3)) 
    self.p = zeros((6,3))
    self.nunPos = zeros((6,)) # x y phi theta C Z
    self.pidGains = zeros((6,))
    self.la = zeros((6,))
    self.ls = zeros((6,))
    
    self.ser = serial.Serial(serialInterface, serialSpeed, timeout = serialTimeout, writeTimeout = 0)
    #self.ser = open('data.txt','r')
    
    self.timerUpdate = QtCore.QTimer()
    self.timerUpdate.timeout.connect(self.update)
    self.timerUpdate.start(serialPeriod)
    self.log("Timer started")
    
    self.dsbP.valueChanged.connect(self.updateControlerGains)
    self.dsbI.valueChanged.connect(self.updateControlerGains)
    self.dsbD.valueChanged.connect(self.updateControlerGains)
    self.sendPID = True
  
  def updateControlerGains(self, gp):
    if self.sendPID:
      print("updateControlerGains")
      self.pidGains[0] = self.dsbP.value()
      self.pidGains[1] = self.dsbI.value()
      self.pidGains[2] = self.dsbD.value()
      self.pidGains[3] = self.dsbP.value()
      self.pidGains[4] = self.dsbI.value()
      self.pidGains[5] = self.dsbD.value()
      self.sendData('cg')
 
  def setupGui(self):
    self.setFixedSize(1190,780)
    self.setWindowTitle('Stewart platform control center')
    
    centralArea = QtGui.QWidget()
    lay0 = QtGui.QHBoxLayout()
    
    lay1 = QtGui.QVBoxLayout()
    
    # Layout 11
    lay11 = QtGui.QGridLayout()
    
    self.pitchCompass = Qwt.QwtCompass()
    self.pitchCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    self.pitchCompass.setValue(90)
    
    self.rollCompass = Qwt.QwtCompass()
    self.rollCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    self.rollCompass.setValue(0)
    
    self.yawCompass = Qwt.QwtCompass()
    self.yawCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    self.yawCompass.setValue(0)
    
    self.figBall = Figure(figsize=(210, 170), dpi=120)
    self.axBall = self.figBall.add_subplot(111)
    fcBall = FigureCanvas(self.figBall)
    
    lay11.addWidget(self.rollCompass, 0, 0)
    lay11.addWidget(self.yawCompass, 0, 1)
    lay11.addWidget(fcBall, 1, 0)
    lay11.addWidget(self.pitchCompass, 1, 1)
    
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
      self.ths[i].setEnabled(False)
    
    lay12.addWidget(self.ths[4],0,1)
    lay12.addWidget(self.ths[3],0,2)
    lay12.addWidget(self.ths[5],2,0)
    lay12.addWidget(self.ths[2],2,3)
    lay12.addWidget(self.ths[0],4,1)
    lay12.addWidget(self.ths[1],4,2)
    
    self.lcds = {}
    for i in range(6):
      self.lcds[i] = QtGui.QLCDNumber()
      self.lcds[i].setSegmentStyle(QtGui.QLCDNumber.Flat)
    
    lay12.addWidget(self.lcds[4],1,1)
    lay12.addWidget(self.lcds[3],1,2)
    lay12.addWidget(self.lcds[5],3,0)
    lay12.addWidget(self.lcds[2],3,3)
    lay12.addWidget(self.lcds[0],5,1)
    lay12.addWidget(self.lcds[1],5,2)
    
    lay1.addLayout(lay11)
    lay1.addLayout(lay12)
    lay1.setStretchFactor(lay11,1)
    lay1.setStretchFactor(lay12,1)
    
    # Layout 2
    lay2 = QtGui.QHBoxLayout()
    
    
    # Layout 21
    lay21 = QtGui.QVBoxLayout()

    self.fig3d = Figure(figsize=(210, 170), dpi=120)
    self.axfig3d = self.fig3d.add_subplot(111, projection='3d', aspect='equal')
    self.axfig3d.set_axis_off()
    fc3d = FigureCanvas(self.fig3d)
    
    self.fig2d = Figure(figsize=(210, 170), dpi=120)
    self.axfig2d = self.fig2d.add_subplot(111)
    fc2d = FigureCanvas(self.fig2d)

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
    labelP4 = QtGui.QLabel('Phi')
    labelP5 = QtGui.QLabel('Theta')
    labelP6 = QtGui.QLabel('Psi')
    
    labelN1 = QtGui.QLabel('X')
    labelN2 = QtGui.QLabel('Y')
    labelN3 = QtGui.QLabel('Phi')
    labelN4 = QtGui.QLabel('Theta')
    labelN5 = QtGui.QLabel('But C')
    labelN6 = QtGui.QLabel('But Z')
    
    self.lcdP = {}
    self.lcdN = {}
    for i in range(6):
      self.lcdP[i] = QtGui.QLCDNumber()
      self.lcdN[i] = QtGui.QLCDNumber()
      self.lcdP[i].setSegmentStyle(QtGui.QLCDNumber.Flat)
      self.lcdN[i].setSegmentStyle(QtGui.QLCDNumber.Flat)
    
    groupP1.addRow(labelP1, self.lcdP[0])
    groupP1.addRow(labelP2, self.lcdP[1])
    groupP1.addRow(labelP3, self.lcdP[2])    
    groupP2.addRow(labelP4, self.lcdP[3])
    groupP2.addRow(labelP5, self.lcdP[4])
    groupP2.addRow(labelP6, self.lcdP[5])
    
    groupN1.addRow(labelN1, self.lcdN[0])
    groupN1.addRow(labelN2, self.lcdN[1])
    groupN1.addRow(labelN5, self.lcdN[4])    
    groupN2.addRow(labelN3, self.lcdN[2])
    groupN2.addRow(labelN4, self.lcdN[3])
    groupN2.addRow(labelN6, self.lcdN[5])
    
    groupPl.addLayout(groupP1)
    groupPl.addLayout(groupP2)
    groupNl.addLayout(groupN1)
    groupNl.addLayout(groupN2)
    
    groupP.setLayout(groupPl)
    groupN.setLayout(groupNl)

    lay21.addWidget(fc3d)
    lay21.addWidget(fc2d)
    lay21.addWidget(groupP)
    lay21.addWidget(groupN)
    lay21.setStretchFactor(fc3d,2)
    lay21.setStretchFactor(fc2d,2)
    lay21.setStretchFactor(groupP,1)
    lay21.setStretchFactor(groupN,1)
  
    
    # Layout 22
    lay22 = QtGui.QVBoxLayout()
    lay221 = QtGui.QGridLayout()
    self.dsbP = QtGui.QDoubleSpinBox()
    self.dsbI = QtGui.QDoubleSpinBox()
    self.dsbD = QtGui.QDoubleSpinBox()
    
    self.dsbP.setSingleStep(0.01)
    self.dsbI.setSingleStep(0.01)
    self.dsbD.setSingleStep(0.01)
    
    self.dsbP.setKeyboardTracking(False)
    self.dsbI.setKeyboardTracking(False)
    self.dsbD.setKeyboardTracking(False)
    
    lay221.addWidget(self.dsbP,0,1)
    lay221.addWidget(self.dsbI,1,1)
    lay221.addWidget(self.dsbD,2,1)
    
    self.lcdpid = {}
    for i in range(3):
      self.lcdpid[i] = QtGui.QLCDNumber()
      self.lcdpid[i].setSegmentStyle(QtGui.QLCDNumber.Flat)
      
    lay221.addWidget(self.lcdpid[0],0,2)
    lay221.addWidget(self.lcdpid[1],1,2)
    lay221.addWidget(self.lcdpid[2],2,2)
    
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
    
    self.tedebug = QtGui.QPlainTextEdit()
    
    lay3.addLayout(lay31)
    lay3.addLayout(lay32)
    lay3.addLayout(lay33)
    lay3.addWidget(self.tedebug)
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
    
  def test(self):
    self.tedebug.setPlainText("test")
  
  def log(self, txt):
    self.tedebug.moveCursor(QtGui.QTextCursor.End)
    self.tedebug.insertPlainText(txt + '\n')
    self.tedebug.ensureCursorVisible()
  
  def update(self):
    #self.log("Reading Serial...")
    #self.log(str(self.ser.inWaiting()))
    msg = self.ser.readline().decode('ascii')
    if len(msg) > 5:
      print("<" + msg)
      self.update_data(msg)
    self.update_gui()

  def sendData(self, msgType):
    msg = None

    if msgType == 'cg':
      # Controler gains PID for both directions
      msg = "cg %.5f %.5f %.5f %.5f %.5f %.5f \n" % (self.pidGains[0], self.pidGains[1], self.pidGains[2], self.pidGains[3], self.pidGains[4], self.pidGains[5])

    if msg :
      self.log(">" + msg)
      self.ser.write(msg.encode('ascii'))

  def update_data(self, msg):
    """
    The data strings are always arranged like
    XX .5f .5f .5f .5f .5f .5f
    where XX is a data code
    
    Angles are in degrees
    """
    data = msg.split()
    
    if len(data) == 7:
      if data[0] == 'SA':
        # Servo angles
        for i in range(6):
          self.alpha[i] = float(data[i+1])
      elif data[0] == 'BS':
        # Ball state x y vx vy ax ay
        for i in range(6):
          self.ballState[i] = float(data[i+1])
      elif data[0] == 'PP':
        # Platform position x y z phi theta psi
        for i in range(6):
          self.platPos[i] = float(data[i+1])
      elif data[0] == 'NP':
        # Nunchuk position  x y phi theta C Z
        for i in range(6):
          self.nunPos[i] = float(data[i+1])
      elif data[0] == 'CG':
        # Controler gains PID for both directions
        for i in range(6):
          self.pidGains[i] = float(data[i+1])  
      
      elif data[0] == 'PX':
        # X-values of the platform attachement points (P points)
        for i in range(6):
          self.p[i][0] = float(data[i+1])  
      elif data[0] == 'PY':
        # Y-values of the platform attachement points (P points)
        for i in range(6):
          self.p[i][1] = float(data[i+1])  
      elif data[0] == 'PZ':
        # Z-values of the platform attachement points (P points)
        for i in range(6):
          self.p[i][2] = float(data[i+1])  
      elif data[0] == 'BX':
        # X-values of the base attachement points (P points)
        for i in range(6):
          self.b[i][0] = float(data[i+1])  
      elif data[0] == 'BY':
        # Y-values of the base attachement points (P points)
        for i in range(6):
          self.b[i][1] = float(data[i+1])  
      elif data[0] == 'BZ':
        # Z-values of the base attachement points (P points)
        for i in range(6):
          self.b[i][2] = float(data[i+1])  
      elif data[0] == 'BT':
        # beta angles
        for i in range(6):
          self.beta[i] = float(data[i+1])  
      elif data[0] == 'LS':
        # length of the push rods 
        for i in range(6):
          self.ls[i] = float(data[i+1])  
      elif data[0] == 'LA':
        # length of the servo arms
        for i in range(6):
          self.la[i] = float(data[i+1])
    else:
      print("Error in the length of the message")

  def update_gui(self):
    #self.axBall.arrow( self.ballState[0], self.ballState[1], 
    #                   self.ballState[2], self.ballState[3], 
    #                   fc="k", ec="k", head_width=0.05, head_length=0.1, 
    #                   length_includes_head=True)
    
    for i in range(6):
      self.ths[i].setValue(int(self.alpha[i]))
      self.lcds[i].display(self.alpha[i])
      self.lcdP[i].display(self.platPos[i])
      self.lcdN[i].display(self.nunPos[i])
      
    self.pitchCompass.setValue(self.platPos[3])
    self.rollCompass.setValue(self.platPos[4])
    
    #self.sendPID = False
    self.lcdpid[0].display(self.pidGains[0])
    self.lcdpid[1].display(self.pidGains[1])
    self.lcdpid[2].display(self.pidGains[2])
    #self.sendPID = True
    
    #self.upate_fc3d()
    
  def upate_fc3d(self):
    vec_s = zeros((6,3)) # push rod
    a = zeros((6,3))
    q = zeros((6,3))
    vec_l = zeros((6,3))
    
    x = self.platPos[0]
    y = self.platPos[1]
    z = self.platPos[2]
    phi = radians(self.platPos[3])
    theta = radians(self.platPos[4])
    psi = radians(self.platPos[5])
    
    
    for i in range(6):
      vec_l[i][0] = cos(psi)*cos(theta) * self.p[i][0] + \
                    (-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi)) * self.p[i][1] + \
                    (sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)) * self.p[i][2] \
                    - self.b[i][0] + x
      vec_l[i][1] = sin(psi)*cos(theta) * self.p[i][0] + \
                    (cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi)) * self.p[i][1] + \
                    (-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi)) * self.p[i][2] \
                    - self.b[i][1] + y
      vec_l[i][2] = -sin(theta) * self.p[i][0] + \
                    cos(theta)*sin(phi) * self.p[i][1] + \
                    cos(theta)*cos(phi) * self.p[i][2] \
                    - self.b[i][2] + z
    
    # Some vectors
    for i in range(6):
      alpha = radians(self.alpha[i])
      beta = radians(self.beta[i])
      
      
      a[i,0] = self.la[i] * cos(alpha) * cos(beta) + self.b[i][0]
      a[i,1] = self.la[i] * cos(alpha) * sin(beta) + self.b[i][1]
      a[i,2] = self.la[i] * sin(alpha)             + self.b[i][2]
      
      q[i][0] = self.b[i][0] + vec_l[i][0]
      q[i][1] = self.b[i][1] + vec_l[i][1]
      q[i][2] = self.b[i][2] + vec_l[i][2]
      
      vec_s[i][0] = q[i][0] - a[i][0]
      vec_s[i][1] = q[i][1] - a[i][1]
      vec_s[i][2] = q[i][2] - a[i][2]
    
    for i in range(6):

      #sevo horn
      x_vect = (self.b[i][0], a[i][0])
      y_vect = (self.b[i][1], a[i][1])
      z_vect = (self.b[i][2], a[i][2])
      self.axfig3d.plot(x_vect,y_vect,'ko-', zs=z_vect, ms=3)
      
      #horn platform
      x_vect = (a[i][0], q[i][0])
      y_vect = (a[i][1], q[i][1])
      z_vect = (a[i][2], q[i][2])
      self.axfig3d.plot(x_vect,y_vect,'ko-', zs=z_vect, ms=3)

      #base
      x_vect = (self.b[i][0], self.b[(i+1)%6][0])
      y_vect = (self.b[i][1], self.b[(i+1)%6][1])
      z_vect = (self.b[i][2], self.b[(i+1)%6][2])
      self.axfig3d.plot(x_vect,y_vect,'g--', zs=z_vect)

      #platform
      x_vect = (q[i][0], q[(i+1)%6][0])
      y_vect = (q[i][1], q[(i+1)%6][1])
      z_vect = (q[i][2], q[(i+1)%6][2])
      self.axfig3d.plot(x_vect,y_vect,'r--', zs=z_vect)

      self.axfig3d.set_xlabel("X")
      self.axfig3d.set_ylabel("Y")
      self.axfig3d.set_label("Z")
      self.axfig3d.set_zlim3d(0)
    self.draw()
      
      
      
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())
