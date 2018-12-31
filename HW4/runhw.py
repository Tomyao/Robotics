# import the necessary packages
import requests
import time
import math
import http.client
from math import atan2, sqrt, sin, cos
import numpy as np

# set the correct host and port here
HOST = '192.168.50.3'
PORT = '8000'

BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

def __request__(url, times=10):
    for x in range(times):
        try:
            requests.get(url)
            return 0
        except :
            print("Connection error, try again")
    print("Abort")
    exit()

def run_action(cmd):
    # set the url include action information
    url = BASE_URL + 'run/?action=' + cmd
    #print('url: %s'% url)
    # post request with url
    __request__(url)

def run_speed(speed):
    # Set set-speed url
    url = BASE_URL + 'run/?speed=' + speed
    #print('url: %s'% url)
    # Set speed
    __request__(url)

def drive_forward():
    run_action('fwturn:90')
    run_speed('100')
    run_action('forward_custom')

def rotate_left():
    run_action('fwturn:65')
    run_speed('80')
    run_action('forward_custom')
    time.sleep(0.1)
    run_action('fwturn:115')
    run_action('backward_custom')

def rotate_right():
    run_action('fwturn:115')
    run_speed('80')
    run_action('forward_custom')
    time.sleep(0.1)
    run_action('fwturn:65')
    run_action('backward_custom')

def get_angle_difference(angle1, angle2):
    angle1 = angle1 % (2 * np.pi)       # force in range [0, 2 pi)
    angle2 = angle2 % (2 * np.pi)       # force in range [0, 2 pi)
    
    diff1 = abs(angle1 - angle2)
    
    angle1_prime = angle1
    angle2_prime = angle2
    
    if angle1 > np.pi:              # move to [-pi, pi)
        angle1_prime -= 2 * np.pi
    if angle2 > np.pi:              # move to [-pi, pi)
        angle2_prime -= 2 * np.pi

    diff2 = abs(angle1_prime - angle2_prime)

    if diff1 < diff2:
        return [angle1, angle2, diff1]

    return [angle1_prime, angle2_prime, diff2]

def rotate(curr_pose, my_dest):
    # first calculate angle from current position to destination
    wanted_angle = atan2(my_dest[1] - curr_pose[1], my_dest[0] - curr_pose[0])

    wanted_angle = wanted_angle % (2 * np.pi)     # force in range [0, 2 pi)
    if wanted_angle > np.pi:                      # move to [-pi, pi)
        wanted_angle -= 2 * np.pi
    
    curr_angle = curr_pose[2] % (2 * np.pi)     # force in range [0, 2 pi)
    if curr_angle > np.pi:                      # move to [-pi, pi)
        curr_angle -= 2 * np.pi

    print ("curr angle: " + str(curr_angle*180/math.pi))
    print ("wanted angle: " + str(wanted_angle*180/math.pi))

    my_epsilon = 0.1
    
    while get_angle_difference(curr_angle, wanted_angle)[2] > my_epsilon:
        time.sleep(0.2)
        temp_angle = get_angle_difference(curr_angle, wanted_angle)
        if temp_angle[0] > temp_angle[1]:
            rotate_right()
            curr_pose[2] -= 0.062 # calibrate angle
        else:
            rotate_left()
            curr_pose[2] += 0.062 # calibrate angle
        curr_angle = curr_pose[2]

    if get_angle_difference(curr_angle, wanted_angle)[2] > 0.062*0.67: # calibrate angle
        time.sleep(0.2)
        temp_angle = get_angle_difference(curr_angle, wanted_angle)
        if temp_angle[0] > temp_angle[1]:
            rotate_right()
            curr_pose[2] -= 0.062 # calibrate angle
        else:
            rotate_left()
            curr_pose[2] += 0.062 # calibrate angle

def forward(curr_pose, my_dest):
    delta = 0.1
    vel = 0.35       # at speed of 100 out of 100

    distance_left = sqrt((curr_pose[0]-my_dest[0])**2 + (curr_pose[1]-my_dest[1])**2)

    print ("wanted distance: " + str(distance_left))
    
    my_epsilon = vel * delta * 0.67
    
    while distance_left > my_epsilon:
        time.sleep(0.2)
        drive_forward()
        curr_pose[0] += (vel * cos(curr_pose[2]) * delta)
        curr_pose[1] += (vel * sin(curr_pose[2]) * delta)
        old_distance_left = distance_left
        distance_left = sqrt((curr_pose[0]-my_dest[0])**2 + (curr_pose[1]-my_dest[1])**2)
        # check for overshooting the destination
        if distance_left > old_distance_left:
            break

# directs the PiCar to run on a path composed of endpoints indicating straight lines
# starts at pose mystart
def run_path(mystart, mypath):
    curr_pose = mystart[:]

    for dest in mypath:
        rotate(curr_pose, dest)
        forward(curr_pose, dest)
        curr_pose[0] = dest[0]
        curr_pose[1] = dest[1]

def main():
    # generated using voronoi_graph.py
    voronoi_path = [[0.0335, -0.1115], [0.39, -0.1925], [0.7, -0.1925], [1.0216, -0.1266], [1.09, 0.2], [1.05, 0.38], [1.09, 0.51], [1.0141, 0.8541], [0.7, 0.915], [0.5, 0.8]]
    # generated using visibility_graph.py
    visibility_path = [[0.83,0.07], [1.05,0.38], [0.83,0.64], [0.5,0.8]]
    
    start_pos = [0,0,0]
    
    #run_path(start_pos, voronoi_path)
    run_path(start_pos, visibility_path)

if __name__ == "__main__":
    main()
