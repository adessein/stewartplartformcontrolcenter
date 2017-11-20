#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 21:00:10 2017

@author: arnaud
"""

from PyQt4 import QtCore, QtGui, uic
import PyQt4.Qwt5 as Qwt
from sys import argv
import sys


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
    lay21 = QtGui.QVBoxLayout()
    frame3d = QtGui.QFrame()
    frame2d = QtGui.QFrame()
    groupP = QtGui.QGroupBox()
    groupN = QtGui.QGroupBox()
    groupP.setTitle('Platform')
    groupN.setTitle('Nunchuck')
    
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

    lay21.addWidget(frame2d)
    lay21.addWidget(frame3d)
    lay21.addWidget(groupP)
    lay21.addWidget(groupN)
    lay21.setStretchFactor(frame2d,2)
    lay21.setStretchFactor(frame3d,2)
    lay21.setStretchFactor(groupP,1)
    lay21.setStretchFactor(groupN,1)
  
    
    lay22 = QtGui.QVBoxLayout()
    
    lay2.addLayout(lay21)
    lay2.addLayout(lay22)
    
    lay3 = QtGui.QVBoxLayout()
    
    lay0.addLayout(lay1)
    lay0.addLayout(lay2)
    lay0.addLayout(lay3)
    lay0.setStretchFactor(lay1,2)
    lay0.setStretchFactor(lay2,2)
    lay0.setStretchFactor(lay3,1)
    
    centralArea.setLayout(lay0)
    self.setCentralWidget(centralArea)
    
        
if __name__ == "__main__":
    app = QtGui.QApplication(argv)
    mainWindow = MainWindow()
    mainWindow.show()
    sys.exit(app.exec_())
