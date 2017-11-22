#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 21:14:18 2017

@author: arnaud
"""

from control import *
from math import degrees,radians, cos, sin, tan, acos, asin, atan, pi, sqrt
from matplotlib import pyplot as plt
from numpy import matrix, power
from numpy.linalg import norm

g = 9.81

class Arduino():
  def __init__(self):
    pass
  
  def start(self):
    self.setup()
    while True:
      self.loop()
  
  def setup(self):
    pass
  
  def loop(self):
    pass
  
class ServosAndPlatform():
  def __init__(self):
    pass
  
class Ball():
  def __init__(self):
    pass
    