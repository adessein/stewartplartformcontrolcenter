# -*- coding: utf-8 -*-
"""
Created on Sat Apr 15 20:55:59 2017

@author: DESSE00A

x1, x2 and x3 correspond to the 0th, 1st and 2nd derivative of the state x
If x1 is position, then x2 is speed and x3 is acceleration

"""

from math import copysign

class State:
    def __init__(self, x1, x2, x3):
        self.x1 = x1
        self.x2 = x2
        self.x3 = x3
    def set_x1(self, p):
        self.x1 = p
    def set_x2(self, p):
        self.x2 = p
    def set_x3(self, p):
        self.x3 = p

class Controller:
    def __init__(self, **argv):
        pass
    def calculate_command(self, error_state):
        pass

class Actuator:
    def __init__(self, init_state):
        self.min = None
        self.max = None
        self.state = init_state
    def set_limits(self, mini, maxi):
        self.min = mini
        self.max = maxi
    def set_x1(self, x1_c, dt):
        # Check if the change in state does not exceed the maxium change rate        
        if self.max.x2 is not None and (abs(x1_c-self.state.x1)/dt > self.max.x2):
            # limited_x1 is the value x coudl reach in dt with the limitation on x2            
            limited_x1 = self.state.x1 + copysign(self.max.x2 * dt, x1_c-self.state.x1)
            self.state.set_x1(limited_x1)
        else :
            self.state.set_x1(x1_c)
        return self.state.x1 
    def set_x2(self, x2_c):
        # TODO : case when x3 is None
        # Check if the change in speed does not exceed the maxium change rate        
        if abs(x2_c-self.state.xc)/dt > self.max.x3:
            # limited_x2 is the value x2 could reach in dt with the limitation on x3
            limited_x2 = self.state.x2 + copysign(self.max.x3 * dt, x2_c-self.state.x2)
            self.state.set_x2(limited_x2)  
        else:
            self.state.set.x2(x2_c)
        return self.state.x2
    def set_x3(self, x3_c):
        # TODO : No limitation on change of x3 for the moment.
        self.state.set_x3(x3_c)
        return self.state.x3

class System:
    def __init__(self, init_state):
        self.act = None
        self.ctl = None
        self.min = None
        self.max = None
        self.state = init_state
    def set_limits(self, mini, maxi):
        self.min = mini
        self.max = maxi
    def set_actuator(self, act):
        self.act = act
    def set_controller(self, ctl):
        self.ctl = ctl
    def set_x1(self, x1_c, dt):
        # TODO : case when x2 is None
        # Check if the change in state does not exceed the maxium change rate        
        if abs(x1_c-self.state.x1)/dt > self.max.x2:
            # limited_x1 is the value x could reach in dt with the limitation on x2            
            limited_x1 = self.state.x1 + copysign(self.max.x2 * dt, x1_x-self.state.x1)
            self.state.set_x1(limited_x1)
            self.state.set_x2(0)
        else:
            self.state.set_x1(x1_c)
        return self.state.x1
    def set_x2(self, x2_c, dt):
        # TODO : case when x3 is None
        # Check if the change in speed does not exceed the maxium change rate        
        if abs(x2_c-self.state.x2)/dt > self.max.x3:
            # limited_x2 is the value x2 could reach in dt with the limitation on x3
            limited_x2 = self.state.x2 + copysign(self.max.x3 * dt, x2_x-self.state.x2)
            self.state.set_x2(limited_x2)   
        else:
            self.state.set_x2(x2_c)
        return self.state.x2
    def set_x3(self, x3_c):
        # TODO : No limitation on change of x3 for the moment.
        self.state.set_x3(x3_c)    
        self.state.x3
    def step(self, t):
        pass
