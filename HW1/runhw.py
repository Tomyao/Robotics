import requests
import time
import math

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

# given a rho, alpha, beta, gives instructions until rho becomes quite small
def move_to_waypoint(rho, alpha, beta):
    while rho > 0.001:
        velocity = k_rho * rho
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
        # adjust angle to match 90 degrees being forward and round to nearest degree
        car_angle = int(round(90 - steer_angle*180/math.pi))
        # make car run for 0.1 seconds
        delta_time = 0.1
        run_action('fwturn:' + str(car_angle))
        run_speed(str(car_velocity))
        run_action('forward')
        time.sleep(delta_time)
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
    run_action('stop')

# read in waypoints
mywaypoints = [[0,0,0], [1,0,0], [1,2,math.pi], [0,0,0]]
with open("waypoints.txt", "r") as filestream:
    for line in filestream:
        currentline = [float(n.rstrip()) for n in line.split(",")]
        mywaypoints.append(currentline)

print (mywaypoints)

# 0,0,0
# 1,0,0
# 1,1,1.57
# 2,1,0
# 2,2,-1.57
# 1,1,-.78
# 0,0,0

# start at pose 0,0,0
curr_pose = [0,0,0]

# doing scaling on x and y coordinates
for b in mywaypoints:
    b[0] = scale*b[0]
    b[1] = scale*b[1]
curr_pose[0] = scale*curr_pose[0]
curr_pose[1] = scale*curr_pose[1]

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
    # update current pose
    curr_pose = waypoint
