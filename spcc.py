#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 21:00:10 2017

Integration of Matplotlib in Qt
https://matplotlib.org/examples/user_interfaces/embedding_in_qt4.html

PyQt4 doc
http://pyqt.sourceforge.net/Docs/PyQt4/



@author: arnaud
"""

from PyQt4 import QtCore, QtGui, uic
import PyQt4.Qwt5 as Qwt
from sys import argv
import sys
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


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
    
    pitchCompass = Qwt.QwtCompass()
    pitchCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    
    rollCompass = Qwt.QwtCompass()
    rollCompass.setNeedle(Qwt.QwtCompassMagnetNeedle(Qwt.QwtCompassMagnetNeedle.ThinStyle))
    
    haDial = Qwt.QwtDial()
    haDial.setMode(Qwt.QwtDial.RotateScale)
    
    figBall = Figure(figsize=(210, 170), dpi=120)
    fcBall = FigureCanvas(figBall)
    
    lay11.addWidget(pitchCompass, 0, 0)
    lay11.addWidget(fcBall, 0, 1)
    lay11.addWidget(haDial, 1, 0)
    lay11.addWidget(rollCompass, 1, 1)
    
    lay11.setColumnStretch(0,1)
    lay11.setColumnStretch(1,1)
    lay11.setRowStretch(0,1)
    lay11.setRowStretch(1,1)
    
    # Layout 12
    lay12 = QtGui.QGridLayout()
    
    ths = {}
    for i in range(6):
      ths[i] = QtGui.QSlider()
      ths[i].setMinimum(-90)
      ths[i].setMaximum(90)
      ths[i].setPageStep(1)
      ths[i].setTickPosition(QtGui.QSlider.TicksBothSides)
      ths[i].setTickInterval(15)
      ths[i].setMinimumSize(38,111)
      ths[i].setMaximumSize(38,111)
    
    lay12.addWidget(ths[0],0,1)
    lay12.addWidget(ths[1],0,2)
    lay12.addWidget(ths[2],1,0)
    lay12.addWidget(ths[3],1,3)
    lay12.addWidget(ths[4],2,1)
    lay12.addWidget(ths[5],2,2)
    
    lay1.addLayout(lay11)
    lay1.addLayout(lay12)
    lay1.setStretchFactor(lay11,1)
    lay1.setStretchFactor(lay12,1)
    
    # Layout 2
    lay2 = QtGui.QHBoxLayout()
    
    
    # Layout 21
    lay21 = QtGui.QVBoxLayout()

    fig3d = Figure(figsize=(210, 170), dpi=120)
    fc3d = FigureCanvas(fig3d)
    
    figprog = Figure(figsize=(210, 170), dpi=120)
    fcprog = FigureCanvas(fig3d)

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
    
    lcdP = {}
    lcdN = {}
    for i in range(6):
      lcdP[i] = QtGui.QLCDNumber()
      lcdN[i] = QtGui.QLCDNumber()
    
    groupP1.addRow(labelP1, lcdP[0])
    groupP1.addRow(labelP2, lcdP[1])
    groupP1.addRow(labelP3, lcdP[2])    
    groupP2.addRow(labelP4, lcdP[3])
    groupP2.addRow(labelP5, lcdP[4])
    groupP2.addRow(labelP6, lcdP[5])
    
    groupN1.addRow(labelN1, lcdN[0])
    groupN1.addRow(labelN2, lcdN[1])
    groupN1.addRow(labelN3, lcdN[2])    
    groupN2.addRow(labelN4, lcdN[3])
    groupN2.addRow(labelN5, lcdN[4])
    groupN2.addRow(labelN6, lcdN[5])
    
    groupPl.addLayout(groupP1)
    groupPl.addLayout(groupP2)
    groupNl.addLayout(groupN1)
    groupNl.addLayout(groupN2)
    
    groupP.setLayout(groupPl)
    groupN.setLayout(groupNl)

    lay21.addWidget(fc3d)
    lay21.addWidget(fcprog)
    lay21.addWidget(groupP)
    lay21.addWidget(groupN)
    lay21.setStretchFactor(fc3d,2)
    lay21.setStretchFactor(fcprog,2)
    lay21.setStretchFactor(groupP,1)
    lay21.setStretchFactor(groupN,1)
  
    
    # Layout 22
    lay22 = QtGui.QVBoxLayout()
    lay221 = QtGui.QFormLayout()
    dsbP = QtGui.QDoubleSpinBox()    
    dsbI = QtGui.QDoubleSpinBox()
    dsbD = QtGui.QDoubleSpinBox()
    lay221.addRow("P", dsbP)
    lay221.addRow("I", dsbI)
    lay221.addRow("D", dsbD)
    
    tbTargetList = QtGui.QTableView()
    
    lay22.addLayout(lay221)
    lay22.addWidget(tbTargetList)
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
    
    pbAutoMan = QtGui.QPushButton("Automatic")
    pbNunGui = QtGui.QPushButton("Nunchuck")
    pbPlatSens = QtGui.QPushButton("Platform")
    pbAutoMan.setCheckable(True)
    pbNunGui.setCheckable(True)
    pbPlatSens.setCheckable(True)
    
    #pbAutoManIcon = QtGui.QIcon()
    #pbAutoManIcon.addPixmap(QtGui.QPixmap('auto.png'), QtGui.QIcon.Normal, QtGui.QIcon.On)
    #pbAutoManIcon.addPixmap(QtGui.QPixmap('manual.png'), QtGui.QIcon.Normal, QtGui.QIcon.Off)
    #pbAutoMan.setIcon(pbAutoManIcon)

    #pbAutoMan.setIconSize(QSize(pbAutoMan.size()
    
    for ts in [pbAutoMan, pbNunGui, pbPlatSens]:
      lay31.addWidget(ts)
      
    pbCenter = QtGui.QPushButton("Centre")
    pbCircle = QtGui.QPushButton("Circle")
    pbSquare = QtGui.QPushButton("Square")
    pbTriangle = QtGui.QPushButton("Triangle")
    
    lay32.addWidget(pbCenter,0,0)
    lay32.addWidget(pbCircle,0,1)
    lay32.addWidget(pbSquare,1,0)
    lay32.addWidget(pbTriangle,1,1)
    
    sls = {}
    for i in range(6):
      sls[i] = QtGui.QSlider()
      sls[i].setMinimum(-90)
      sls[i].setMaximum(90)
      sls[i].setPageStep(1)
      sls[i].setTickPosition(QtGui.QSlider.TicksBothSides)
      sls[i].setTickInterval(15)
      sls[i].setMinimumSize(28,145)
      sls[i].setMaximumSize(28,145)
      lay331.addWidget(sls[i],0,i)
      lab = QtGui.QLabel(str(i))
      lab.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
      lay331.addWidget(lab,1,i)
    
    lay331.setRowStretch(6,1)
    
    pbHome = QtGui.QPushButton("Home")
    pbZero = QtGui.QPushButton("0 deg")
    lay332.addWidget(pbHome)
    lay332.addWidget(pbZero)
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
    
        
if __name__ == "__main__":
    #app = QtGui.QApplication(argv)
    mainWindow = MainWindow()
    mainWindow.show()
    #sys.exit(app.exec_())
