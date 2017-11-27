# -*- coding: utf-8 -*-
"""
Simulation of the platform

 
Based on the code Ball_6_dof Created on Sun Apr 16 11:45:20 2017


@author: nonolili
"""

from control import *
from math import degrees,radians, cos, sin, tan, acos, asin, atan, pi, sqrt
from matplotlib import pyplot as plt
from numpy import matrix, power
from numpy.linalg import norm

g = 9.81# * 5/7
inf = float("infinity")

class Point():
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

class PID(Controller):
    def __init__(self, *args, **kwargs):
        Controller.__init__(self)
        self.p = kwargs.get('p', 0)
        self.i = kwargs.get('i', 0)
        self.d = kwargs.get('d', 0)
        self.error = []
        
    def set_pid(self, p, i, d):
        self.p = p
        self.i = p
        self.d = d
        
    def calculate_command(self, error):
        # TODO : error should be a state...
        self.error.append(error)
        # I and D can only be active if I have at least two errors samples
        # For the first step, I only use the proportional
        # TODO : check that is OK
        if len(self.error)>1 :
            command = self.p * self.error[-1] + \
                      self.i * (self.error[-1]+self.error[-2]) + \
                      self.d * (self.error[-1]-self.error[-2])
        else :
            command = self.p * self.error[-1]
        return command            

class Servo(Actuator):
    def __init__(self, init_state):
        Actuator.__init__(self, init_state)
    def set_angle(self, a, dt):
        """ Set servomotor angle to a [deg]
        """
        old_angle = self.state.x1
        new_angle = self.set_x1(a, dt)
        speed = (new_angle-old_angle)/dt
        return new_angle, speed
        
    def angle(self):
        return self.x1        

class Ball_on_rail(System):
    def __init__(self, init_state):
        System.__init__(self, init_state)
    def step(self, target, dt):
        error = target - self.state.x1
        command = self.ctl.calculate_command(error)
        reached_angle, reached_speed = self.act.set_angle(command, dt)
        # Set new position and velocity of the ball according to the new
        # inclinason of the plate
        reached_x1 = self.set_x1(self.state.x1 + dt * self.state.x2 + g/2 * sin(radians(reached_angle))  * dt**2, dt)        
        reached_x2 = self.set_x2(self.state.x2 + g * sin(radians(reached_angle))  * dt, dt)
        # Return data to the main loop for plotting                
        data = {'error' : error, 'command' : command, 
                'servo_angle' : reached_angle, 'servo_speed' : reached_speed,
                'ball_pos' : reached_x1, 'ball_vel' : reached_x2}
        return data
        
        
class Ball_on_plate_2dofs():
    def __init__(self, start_point, plate_length, plate_width, pid_gains, 
                 servo_max_speed):
        # Define two Ball_on_rail systems where the ball has an initial
        # position, speed and acceleration equal to 0
        self.system_x = Ball_on_rail(State(start_point.x, 0, 0))
        self.system_y = Ball_on_rail(State(start_point.y, 0, 0))
        
        self.system_x.set_limits(0, State(plate_length, float("infinity"), inf))
        self.system_y.set_limits(0, State(plate_width, inf, inf))  

        # Define two servos whith zero initial angle, speed and acceleration
        self.servo_x = Servo(State(0, 0, 0))
        self.servo_y = Servo(State(0, 0, 0))    
        
        # Define the limits of the servos        
        self.servo_min_state = State(0, -servo_max_speed, None)
        self.servo_max_state = State(180, servo_max_speed, None)
        self.servo_x.set_limits(self.servo_min_state, self.servo_max_state)
        self.servo_y.set_limits(self.servo_min_state, self.servo_max_state)
        
        self.system_x.set_actuator(self.servo_x)
        self.system_y.set_actuator(self.servo_y)
        
        self.pid_servo_x = PID(p=pid_gains['p'],
                               i=pid_gains['i'],
                               d=pid_gains['d'])
        self.pid_servo_y = PID(p=pid_gains['p'],
                               i=pid_gains['i'],
                               d=pid_gains['d'])  
        
        self.system_x.set_controller(self.pid_servo_x)
        self.system_y.set_controller(self.pid_servo_y)
        

    def set_pid(self, p, i, d):
        self.pid_servo_x.set_pid(p, i, d)
        self.pid_servo_y.set_pid(p, i, d)


    def step(self, target_x, target_y, dt):
        data_x = self.system_x.step(target_x, dt)
        data_y = self.system_y.step(target_y, dt)
        return {'data_x' : data_x, 'data_y' : data_y}
        
class Ball_on_plate_6dofs(Ball_on_plate_2dofs):
    """ This ball on plate still had 2 actuators (servos) but these are 
    somehow virtual. The pitch an roll of the plate are used ot calculate the
    leg length of an exapord platform using 6 servos as actuators.
    Only the reverse kinetiks is used : the forward is not used and thus the 
    limitation in servo angle and speed is not taken into account (only 
    displayed in graphs)
    """

    def __init__(self, start_point, plate_length, plate_width, pid_gains, 
                 servo_max_speed):
        Ball_on_plate_2dofs.__init__(self, start_point, plate_length, 
                                     plate_width, pid_gains, servo_max_speed)
        # 3. Calculate the values of h0 from equation (10)
        # and alpha0 from equation 12
        L0 = 2*a**2
        M0 = 2*a*(xp-xb)

        self.h0 = sqrt(s**2+a**2-(xp-xb)**2-(yp-yb)**2)-zp
        N0 = 2*a*(self.h0+zp)
        self.alpha_0 = asin(L0/sqrt(M0**2+N0**2)) - atan(M0/N0)
        
    def step(self, target_x, target_y, dt):
        data = Ball_on_plate_2dofs.step(self, target_x, target_y, dt)
        # Calculate leg lengths
        ## 4. Input the variables (x, y, z, psi, theta, phi) for the
        ## requiered platform position
        T = matrix([0,0,zp]).T
        psi = radians(0.0) # roll        
        phi = radians(data['data_x']['servo_angle']) # roll
        theta = radians(data['data_y']['servo_angle']) # pitch
        ## 5. Calculate the rotational matrix Rb from eq (1)
        Rb=matrix((
        (cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi)),
        (sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi)),
        (-sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi))
        ))
        ## 6. Calculate the effective lemgths l_i from eq (3)
        ## 7. Calculate the angles alpha_i requiered for each servo from eq (9) 
        l = []
        alpha = []
        for i in range(6):
            ## 6. Calculate the effective lemgths l_i from eq (3)
            l.append(norm(T + Rb * p[i] - b[i]))
            ## 7. Calculate the angles alpha_i requiered for each servo from eq (9)            
            L = l[i]**2-(s**2-a**2)
            M = 2*a*(zp-zb)
            N = 2*a*(cos(beta[i])*(xp-xb)+sin(beta[i])*(yp-yb))
            try : 
                alpha.append(degrees(asin(L/sqrt(M**2+N**2))-atan(N/M)))        
            except ValueError, ve :
                print("Step {}".format(i))
                print("l[{}]= {:.2f} mm".format(i,l[i]*1000))
                print("L = {:.2f} mm".format(L*1000))
                print("b[{}].T= {}".format(i,b[i].T))
                print("N/M = {:.2f}".format(N/M))
                raise(ve)
            
        data['l'] = l
        data['alpha'] = alpha
        return data
        
def circle(i, center, radius, period):
    x = radius * cos(2*pi*i/period) + center.x
    y = radius * sin(2*pi*i/period) + center.y
    return Point(x,y)

def square(i, center, length, period):
    if i<period/4 :
        x = center.x - length/2 + length * i/(period/4)
        y = center.y - length/2
    elif i<period/2:
        x = center.x + length/2
        y = center.y - length/2 + length * (i-period/4)/(period/4)
    elif i<3/4*period:
        x = center.x + length/2 - length * (i-period/2)/(period/4)
        y = center.y + length/2
    else :
        x = center.x - length/2
        y = center.y + length/2 - length * (i-period*3/4)/(period/4)
    return Point(x,y)

def hypocycloid(i, center, R, k, period):
    theta = 2*pi*i/period
    r=R/k
    x=((R-r)*cos(theta)) + r*cos((R-r)/r*theta) + center.x
    y=((R-r)*sin(theta)) - r*sin((R-r)/r*theta) + center.y
    return Point(x,y)

if __name__ == "__main__": 

    Tmax = 5
    
    dt = 0.1
    plate_length = 224.5e-3
    plate_width = 172.5e-3            
    
    center = Point(plate_length/2, plate_width/2)
    start_point_hypercycloid = Point(6e-2+plate_length/2, plate_width/2)    
    start_point_circle = Point(6e-2+plate_length/2, plate_width/2)    
    
    #start_point = start_point_circle    
    start_point = start_point_hypercycloid

    pid_gains = {'p': 1,
                 'i': 0, 
                 'd': 300}
    servo_speed = 60.0/0.17 #deg/sec noload@4.8V
    #servo_speed = 60.0/0.13 #deg/sec noload@6.0V

    
    ## OPTION 2 : Ball_on_plate_6dofs

    a = 50e-3
    c = a/2+a/4*sqrt(2)/2
    d = a/2+3*a/4*sqrt(2)/2
    e = a/2*(1+sqrt(2)) 
    
    # list of vectors p_i (attachement points on platform)
    p0 = matrix([-c,d,0])
    p1 = matrix([c,d,0])
    p2 = matrix([e,-a/4,0])
    p3 = matrix([a/4,-e,0])
    p4 = matrix([-a/4,-e,0])
    p5 = matrix([-e,-a/4,0])
    p = [p0.T, p1.T, p2.T, p3.T, p4.T, p5.T]
    # list of vectors b_i (center of rotation of servo arms)
    b0 = matrix([-a/4,e,0])
    b1 = matrix([a/4,e,0])
    b2 = matrix([d,-c,0])
    b3 = matrix([c,-d,0])
    b4 = matrix([-c,-d,0])
    b5 = matrix([-d,-c,0])
    b = [b0.T, b1.T, b2.T, b3.T, b4.T, b5.T]
    beta = [90,90,-45,-45,-135,-135]
    beta = [radians(a) for a in beta]# list of angles beta_i
    a = 1e-2 # length of the servo arms
    s = 10e-2 # length of the push rods
    xb = 0
    yb = 0
    zb = 0
    xp = 0
    yp = 0
    zp = 10e-2

    bop = Ball_on_plate_6dofs(start_point, plate_length, plate_width,
                              pid_gains, servo_speed)
    ## END OF OPTION 
    
    # Array to contain data to be plotted
    pos_x = []
    pos_y = []
    vel_x = []
    vel_y = []
    command_x = []
    command_y = []
    error_x = []
    error_y = []
    target_x = []
    target_y = []
    time = []
    # 2 virtual servos
    servo_angle_x = []
    servo_angle_y = []
    servo_speed_x = []
    servo_speed_y = []
    # 6 real servos
    servo_angle_0 = []
    servo_angle_1 = []
    servo_angle_2 = []
    servo_angle_3 = []
    servo_angle_4 = []
    servo_angle_5 = []
    # speeds
    servo_speed_0 = [None,]
    servo_speed_1 = [None,]
    servo_speed_2 = [None,]
    servo_speed_3 = [None,]
    servo_speed_4 = [None,]
    servo_speed_5 = [None,]
    
    for i in range(0,int(Tmax/dt)):
        #target = square(i, center, 5e-2, Tmax/dt)
        #target = center
        #target = circle(i, center, 6e-2, Tmax/dt)
        target = hypocycloid(i, center, 6e-2, 6, Tmax/dt)
        
        data = bop.step(target.x, target.y, dt)
        
        pos_x.append(data['data_x']['ball_pos']*100)
        pos_y.append(data['data_y']['ball_pos']*100)
        vel_x.append(data['data_x']['ball_vel']*100)
        vel_y.append(data['data_y']['ball_vel']*100)
        command_x.append(data['data_x']['command'])
        command_y.append(data['data_y']['command'])
        error_x.append(data['data_x']['error']*100)
        error_y.append(data['data_y']['error']*100)
        servo_angle_x.append(data['data_x']['servo_angle'])
        servo_angle_y.append(data['data_y']['servo_angle'])
        servo_speed_x.append(data['data_x']['servo_speed']/servo_speed*100)
        servo_speed_y.append(data['data_y']['servo_speed']/servo_speed*100)
        servo_angle_0.append(data['alpha'][0])
        servo_angle_1.append(data['alpha'][1])
        servo_angle_2.append(data['alpha'][2])
        servo_angle_3.append(data['alpha'][3])
        servo_angle_4.append(data['alpha'][4])
        servo_angle_5.append(data['alpha'][5])
        
        if i :
            servo_speed_0.append((servo_angle_0[i]-servo_angle_0[i-1])/dt/servo_speed*100)
            servo_speed_1.append((servo_angle_1[i]-servo_angle_1[i-1])/dt/servo_speed*100)
            servo_speed_2.append((servo_angle_2[i]-servo_angle_2[i-1])/dt/servo_speed*100)
            servo_speed_3.append((servo_angle_3[i]-servo_angle_3[i-1])/dt/servo_speed*100)
            servo_speed_4.append((servo_angle_4[i]-servo_angle_4[i-1])/dt/servo_speed*100)
            servo_speed_5.append((servo_angle_5[i]-servo_angle_5[i-1])/dt/servo_speed*100)            
        
        time.append(i*dt)
        target_x.append(target.x*100)
        target_y.append(target.y*100)
        
    f1, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2,3, sharex=False, sharey=False)
    ax1.plot(time, error_x, 'r-')
    ax1.plot(time, error_y, 'g-')
    ax1.set_title("Error")
    ax1.set_ylabel("Error [cm]")
    
    ax2.plot(time, servo_angle_x, 'r-', label="Roll")
    ax2.plot(time, servo_angle_y, 'g-', label="Pitch")
    ax2.set_title("Servomotor angle")
    ax2.set_ylabel("Angle [deg]")
    ax2.legend()
    
    ax3.plot(time, pos_x, 'r-', label="Actual x")
    ax3.plot(time, pos_y, 'g-', label="Actual y")
    ax3.plot(time, target_x, 'r--', label="Target x")
    ax3.plot(time, target_y, 'g--', label="Target y")
    ax3.set_title("Position and target")    
    ax3.set_ylabel("Position [cm]")
    
    ax5.plot(time, servo_speed_x, 'r-')
    ax5.plot(time, servo_speed_y, 'g-')
    ax5.set_title("Servomotor speed")
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Angular speed [% max speed]")
        
    ax4.plot(time,  command_x, 'r-')
    ax4.plot(time,  command_y, 'g-')
    ax4.set_title("Command")    
    ax4.set_ylabel("Command [deg]")
    
    ax6.plot(target_x, target_y, 'o', markeredgecolor='r', markerfacecolor='None', label="Target")
    ax6.plot(pos_x, pos_y, 'b+', label="Actual")
    ax6.set_title("Travel")
    ax6.set_xlim([0,plate_length*100])    
    ax6.set_ylim([0,plate_width*100])
    ax6.legend()
    
    
    f2, ((axs1, axs2, axs3), (axs4, axs5, axs6)) = plt.subplots(2,3, sharex=True, sharey=False)
    axs1t = axs1.twinx()    
    axs1.plot(time, servo_angle_0, 'r-')    
    axs1t.plot(time, servo_speed_0, 'g-')
    axs1.set_title("Servomotor 0")
    axs1.set_ylabel("Angle [deg]")
    axs1t.set_ylabel("Speed [% max]")
    
    axs2t = axs2.twinx()
    axs2.plot(time, servo_angle_1, 'r-')
    axs2t.plot(time, servo_speed_1, 'g-')
    axs2.set_title("Servomotor 1")
    axs2.set_ylabel("Angle [deg]")
    axs2t.set_ylabel("Speed [% max]")
    
    axs3t = axs3.twinx()    
    axs3.plot(time, servo_angle_2, 'r-')
    axs3t.plot(time, servo_speed_2, 'g-')
    axs3.set_title("Servomotor 2")
    axs3.set_ylabel("Angle [deg]")
    axs3t.set_ylabel("Speed [% max]")
    
    axs4t = axs4.twinx()    
    axs4.plot(time, servo_angle_3, 'r-')
    axs4t.plot(time, servo_speed_3, 'g-')
    axs4.set_title("Servomotor 3")
    axs4.set_ylabel("Angle [deg]")
    axs4t.set_ylabel("Speed [% max]")
    
    axs5t = axs5.twinx()    
    axs5.plot(time, servo_angle_4, 'r-')
    axs5t.plot(time, servo_speed_4, 'g-')
    axs5.set_title("Servomotor 4")
    axs5.set_ylabel("Angle [deg]")
    axs5t.set_ylabel("Speed [% max]")
    
    axs6t = axs6.twinx()    
    axs6.plot(time, servo_angle_5, 'r-')
    axs6t.plot(time, servo_speed_5, 'g-')
    axs6.set_title("Servomotor 5")
    axs6.set_ylabel("Angle [deg]")
    axs6t.set_ylabel("Speed [% max]")
    
    plt.show()
    
    
