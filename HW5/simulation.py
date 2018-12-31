# import the necessary packages
import time
import math
import numpy as np
from random import randint
from math import sqrt, atan2, tan, cos, sin
from sympy import symbols, Matrix
from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt
import matplotlib.pyplot as plt

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer
    
    def predict(self, u=0):
        self.x = self.move(self.x, u, self.dt)
    
    def check_next(self, u=0):
        return self.move(self.x, u, self.dt)
    
    def move(self, x, u, dt):
        hdg = x[2, 0]
        vel = u[0]
        steering_angle = u[1]
        dist = vel * dt
        
        if steering_angle != 0: # robot is turning
            beta = (dist / self.wheelbase) * tan(steering_angle)
            r = self.wheelbase / tan(steering_angle) # radius
            
            dx = np.array([[-r*sin(hdg) + r*sin(hdg + beta)],
                           [r*cos(hdg) - r*cos(hdg + beta)],
                           [beta]])
        else: # moving in straight line
            dx = np.array([[dist*cos(hdg)],
                           [dist*sin(hdg)],
                           [0]])
        return x + dx

# random subsumption model
def run_picar(start_pose, std_vel, std_steer, std_range, std_bearing, time_wanted):
    delta = 0.1
    D = 0.15     # i measured approx 15cm between back wheels and front wheels
    D2 = 0.08    # i measured approx 8cm between camera and front wheels
    myspeed = 100
    vel = 0.4 * myspeed/100.0   # 0.4 at speed of 100 out of 100
    ekf = RobotEKF(delta, wheelbase=D, std_vel=std_vel, std_steer=std_steer)
    ekf.x = array([start_pose]).T    # x, y, orientation
    ekf.P = np.diag([0, 0, 0])      # no uncertainty at very start
    ekf.R = np.diag([std_range**2, std_bearing**2])
    
    my_steer = 12*math.pi/180   # although the command sent to the PiCar is for 45 degrees, it seems
                                # to be closer this many degrees (away from forward) in reality
    
    # steering command (velocity, steering angle in radians)
    u = array([vel, 0])
    forward_flag = True
    left_flag = False
    right_flag = False
    
    # for plotting course
    xposplot = []
    yposplot = []
    
    # set boundaries here (remember to pad)
    mypad = 0.08
    minx = 0.0 + mypad
    maxx = 5*0.3 + 0.09 - mypad
    miny = 0.0 + mypad
    maxy = 5*0.3 + 0.23 - mypad
    
    # set obstacles here in terms of min and max x and y values (remember to pad)
    obstacles = []
    obstacles.append([0.75 - 0.025 - mypad, 0.75 + 0.025 + mypad, 0.75 - 0.025 - mypad, 0.75 + 0.025 + mypad])
    
    elapsed_time = 0
    elapsed_time_steps = None
    wanted_time_steps = None
    
    while elapsed_time < time_wanted:
        xposplot.append(ekf.x[0,0])
        yposplot.append(ekf.x[1,0])

        # check if next move would result in collision
        will_collide = False
        predicted_position = ekf.check_next(u=u)
        
        predicted_position2 = [None, None, None]    # camera location
        predicted_position2[0] = predicted_position[0] + cos(predicted_position[2])*(D + D2)
        predicted_position2[1] = predicted_position[1] + sin(predicted_position[2])*(D + D2)
        predicted_position2[2] = predicted_position[2]
        
        # collision checks with back wheel
        # check collision with boundary
        if predicted_position[0] < minx or predicted_position[0] > maxx:
            will_collide = True
        elif predicted_position[1] < miny or predicted_position[1] > maxy:
            will_collide = True
        # check collision with obstacles
        for myobs in obstacles:
            if predicted_position[0] > myobs[0] and predicted_position[0] < myobs[1]:
                if predicted_position[1] > myobs[2] and predicted_position[1] < myobs[3]:
                    will_collide = True

        # collision checks with camera
        # check collision with boundary
        if predicted_position2[0] < minx or predicted_position2[0] > maxx:
            will_collide = True
        elif predicted_position2[1] < miny or predicted_position2[1] > maxy:
            will_collide = True
        # check collision with obstacles
        for myobs in obstacles:
            if predicted_position2[0] > myobs[0] and predicted_position2[0] < myobs[1]:
                if predicted_position2[1] > myobs[2] and predicted_position2[1] < myobs[3]:
                    will_collide = True
        
        # if imminent collision, change driving direction and pick left or right at random
        # also randomly choose how many time steps to go left/right before going straight
        if will_collide:
            # change direction
            forward_flag = not forward_flag
            u[0] = -u[0]
            # pick number of time steps
            #wanted_time_steps = randint(0, 5)
            #wanted_time_steps = randint(0, 10)
            wanted_time_steps = randint(0, 25)
            if wanted_time_steps > 0:
                elapsed_time_steps = 0
                # pick left or right
                if randint(0, 1) == 0:
                    u[1] = my_steer
                    left_flag = True
                    right_flag = False
                else:
                    u[1] = -my_steer
                    left_flag = False
                    right_flag = True
            else:
                u[1] = 0
                left_flag = False
                right_flag = False
            continue
        
        # update elapsed time steps
        if elapsed_time_steps != None:
            elapsed_time_steps += 1
            # going straight
            if elapsed_time_steps == wanted_time_steps:
                u[1] = 0
                left_flag = False
                right_flag = False
                elapsed_time_steps = None

        # do internal pose prediction
        ekf.predict(u=u)

        # updating elapsed time
        elapsed_time += 1

    # draw plot of course taken
    plt.plot(xposplot, yposplot, color ='black')
    # draw boundaries
    xplot = [minx - mypad, minx - mypad, maxx + mypad, maxx + mypad, minx - mypad]
    yplot = [miny - mypad, maxy + mypad, maxy + mypad, miny - mypad, miny - mypad]
    plt.plot(xplot, yplot, color ='blue')
    # draw padded boundaries
    xplot = [minx, minx, maxx, maxx, minx]
    yplot = [miny, maxy, maxy, miny, miny]
    plt.plot(xplot, yplot, color ='green')
    # draw obstacles
    for obs in obstacles:
        xplot = [obs[0] + mypad, obs[0] + mypad, obs[1] - mypad, obs[1] - mypad, obs[0] + mypad]
        yplot = [obs[2] + mypad, obs[3] - mypad, obs[3] - mypad, obs[2] + mypad, obs[2] + mypad]
        plt.plot(xplot, yplot, color ='red')
    # draw padded obstacles
    for obs in obstacles:
        xplot = [obs[0], obs[0], obs[1], obs[1], obs[0]]
        yplot = [obs[2], obs[3], obs[3], obs[2], obs[2]]
        plt.plot(xplot, yplot, color ='orange')
    # show plot!
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def main():
    start_pose = [0.3,0.75,0]
    vel_err = 0.03          # error in terms of std dev for velocity
    steer_err = 0.05        # error in terms of std dev for steering angle
    range_err = 0.06        # error in terms of std dev for range measurement
    bearing_err = 0.06      # error in terms of std dev for bearing measurement
    time_wanted = 5000     # how long to run for in terms of timesteps
    
    run_picar(start_pose, vel_err, steer_err, range_err, bearing_err, time_wanted)

if __name__ == "__main__":
    main()
