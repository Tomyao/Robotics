import requests
import time
import math

import numpy as np
import matplotlib.pyplot as plt

# set the correct host and port here
HOST = '192.168.0.17'
PORT = '8000'

BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

max_speed = 0.5 # at speed of 100, i measured approx 50cm/sec
scale = 0.5 # scaling waypoints so that it doesn't take so much physical space to run
L = 0.15 # i measured approx 15cm between back wheels and front wheels

# set some constants here
k_rho = 0.5 # k_rho must be > 0
k_alpha = 1.5 # k_alpha - k_rho must be > 0
k_beta = -1.0 # k_beta must be < 0

def __request__(url, times=10):
    for x in range(times):
        try:
            requests.get(url)
            return 0
        except :
            print("Connection error, try again")
    print("Abort")
    return -1

def run_action(cmd):
    # set the url include action information
    url = BASE_URL + 'run/?action=' + cmd
    print('url: %s'% url)
    # post request with url
    __request__(url)

def run_speed(speed):
    # Set set-speed url
    url = BASE_URL + 'run/?speed=' + speed
    print('url: %s'% url)
    # Set speed
    __request__(url)

test1 = []
test2 = []
test3 = []

# given a rho, alpha, beta, gives instructions until rho becomes quite small
def move_to_waypoint(rho, alpha, beta):
    myx = 0
    myy = 0
    while rho > 0.001:
        #if (rho < 0.05):
        test1.append(rho)
        test2.append(beta)
        test3.append(alpha)
        print ("blah")
        print (rho)
        print (alpha*180/math.pi)
        print (beta*180/math.pi)
        # change alpha and beta to be between 0 -2pi and 2pi
        #alpha = alpha % (2*math.pi)
        #beta = beta % (2*math.pi)
        
        #time.sleep(0.1)
        # we want velocity to be 100
        #velocity = max_speed
        velocity = k_rho * rho
        #k_rho = velocity / rho
        angle = k_alpha*alpha + k_beta*beta
        # calculate steering angle
        steer_angle = math.atan((L/velocity) * angle)
        # adjust speed to nearest integer in terms of car speed
        car_velocity = int(round(velocity/max_speed * 100))
        if car_velocity > 100:
            car_velocity = 100
            velocity = max_speed
        elif car_velocity < 40:
            car_velocity = 40
            velocity = max_speed*0.4
        print ("car_velocity: " + str(car_velocity))
        # adjust angle to match 90 degrees being forward and round to nearest degree
        car_angle = int(round(90 - steer_angle*180/math.pi))
        print ("car_angle: " + str(car_angle))
        # make car run for 0.1 seconds
        delta_time = 0.1
#run_action('stop')
        # update values of rho, alpha, beta
        rho_temp = delta_time * (-velocity*math.cos(alpha))
        alpha_temp = delta_time * (k_rho*math.sin(alpha) - k_alpha*alpha - k_beta*beta)
        beta_temp = delta_time * (-k_rho*math.sin(alpha))
        if math.cos(alpha) < 0:
            rho -= rho_temp
        else:
            rho += rho_temp
        alpha += alpha_temp
        beta += beta_temp

# read in waypoints and scale them
mywaypoints = []
with open("waypoints.txt", "r") as filestream:
    for line in filestream:
        currentline = [scale * float(n.rstrip()) for n in line.split(",")]
        mywaypoints.append(currentline)

print (mywaypoints)

# 0,0,0
# 1,0,0
# 1,1,1.57
# 2,1,0
# 2,2,-1.57
# 1,1,-.78
# 0,0,0

mywaypoints = [[0,0,0]]

# start at pose 0,0,0
curr_pose = [1,1,-0.78]

for b in mywaypoints:
    b[0] = scale*b[0]
    b[1] = scale*b[1]
curr_pose = [scale*a for a in curr_pose]

print (curr_pose)
print (mywaypoints)

# loop thru waypoints
for waypoint in mywaypoints:
    # calculate rho, alpha, beta
    delta_x = waypoint[0] - curr_pose[0]
    delta_y = waypoint[1] - curr_pose[1]
    # if location doesn't change, there is no alpha so skip
    if delta_x == 0 and delta_y == 0:
        continue
    myrho = math.sqrt(delta_x**2 + delta_y**2)
    # if delta_x equals 0, then alpha is 90 degrees minus theta
    myalpha = 90*math.pi/180 - curr_pose[2]
    if delta_x != 0:
        myalpha = math.atan(delta_y/delta_x) - curr_pose[2]
    mybeta = -curr_pose[2] - myalpha
    # make it so that the end pose is 0,0,0 by changing frame of reference
    mybeta = mybeta + waypoint[2]
    # run car
    move_to_waypoint(myrho, myalpha, mybeta)
    # change current pose
    curr_pose = waypoint

ax = plt.subplot(111, projection='polar')
#plt.scatter(test1, test2)
plt.plot(test2, test1)
plt.plot(test3, test1)
plt.show()
