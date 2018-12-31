# import the necessary packages
from pyzbar import pyzbar
import cv2
import requests
import time
import math
import http.client
import numpy as np
import sympy
from random import randint
from math import sqrt, atan2, tan, cos, sin
from sympy import symbols, Matrix
from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import dot, array, sqrt
import matplotlib.pyplot as plt

# set the correct host and port here
HOST = '192.168.1.76'
PORT = '8000'
IMG_PORT = '8080'

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

def queryImage():
    http_data = http.client.HTTPConnection(HOST, IMG_PORT)
    http_data.putrequest('GET', "/?action=snapshot")
    http_data.putheader('Host', HOST)
    http_data.putheader('User-agent', 'python-http.client')
    http_data.putheader('Content-type', 'image/jpeg')
    http_data.endheaders()
    returnmsg = http_data.getresponse()
    
    data = returnmsg.read()
    
    with open("temp.jpeg", "wb") as my_file:
        my_file.write(data)
    
    return cv2.imread("temp.jpeg")

def undistort(my_img, my_mtx, my_dist):
    h,  w = my_img.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(my_mtx,my_dist,(w,h),1,(w,h))
    
    # return undistorted image after processing
    return cv2.undistort(my_img, my_mtx, my_dist, None, newcameramtx)

# moves the car forward to the left at 45 degrees for 0.1 seconds
def forward_left(myspeed):
    run_action('fwturn:45')
    run_speed(str(myspeed))
    run_action('forward_custom')

# moves the car backwards to the left at 45 degrees for 0.1 seconds
def backward_left(myspeed):
    run_action('fwturn:45')
    run_speed(str(myspeed))
    run_action('backward_custom')

# moves the car forward to the right at 45 degrees for 0.1 seconds
def forward_right(myspeed):
    run_action('fwturn:135')
    run_speed(str(myspeed))
    run_action('forward_custom')

# moves the car backwards to the right at 45 degrees for 0.1 seconds
def backward_right(myspeed):
    run_action('fwturn:135')
    run_speed(str(myspeed))
    run_action('backward_custom')

# moves the car forward for 0.1 seconds
def forward_straight(myspeed):
    run_action('fwturn:90')
    run_speed(str(myspeed))
    run_action('forward_custom')

# moves the car backwards for 0.1 seconds
def backward_straight(myspeed):
    run_action('fwturn:90')
    run_speed(str(myspeed))
    run_action('backward_custom')

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
def H_of(x, landmark_pos):
    """ compute Jacobian of H matrix where h(x) computes
        the range and bearing to a landmark for state x """
    
    px = landmark_pos[0]
    py = landmark_pos[1]
    
    hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
    dist = sqrt(hyp)
    
    H = array(
              [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
               [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    
    return H

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
def Hx(x, landmark_pos):
    """ takes a state variable and returns the measurement
        that would correspond to that state.
        """
    px = landmark_pos[0]
    py = landmark_pos[1]
    
    dist = sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
    
    Hx = array([[dist],
                [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return Hx

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
class RobotEKF(EKF):
    def __init__(self, dt, wheelbase, std_vel, std_steer):
        EKF.__init__(self, 3, 2, 2)
        self.dt = dt
        self.wheelbase = wheelbase
        self.std_vel = std_vel
        self.std_steer = std_steer
        
        a, x, y, v, w, theta, time = symbols('a, x, y, v, w, theta, t')
        d = v*time
        beta = (d/w)*sympy.tan(a)
        r = w/sympy.tan(a)
                                             
        self.fxu = Matrix([[x-r*sympy.sin(theta)+r*sympy.sin(theta+beta)],[y+r*sympy.cos(theta)-r*sympy.cos(theta+beta)],[theta+beta]])
                                             
        self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        self.V_j = self.fxu.jacobian(Matrix([v, a]))
                                             
        # save dictionary and it's variables for later use
        self.subs = {x: 0, y: 0, v:0, a:0, time:dt, w:wheelbase, theta:0}
        self.x_x, self.x_y, = x, y
        self.v, self.a, self.theta = v, a, theta
    
    def predict(self, u=0):
        self.x = self.move(self.x, u, self.dt)
        
        self.subs[self.theta] = self.x[2, 0]
        self.subs[self.v] = u[0]
        self.subs[self.a] = u[1]
        
        F = array(self.F_j.evalf(subs=self.subs)).astype(float)
        V = array(self.V_j.evalf(subs=self.subs)).astype(float)
        
        # covariance of motion noise in control space
        M = array([[self.std_vel*u[0]**2, 0],
                   [0, self.std_steer**2]])
                   
        self.P = dot(F, self.P).dot(F.T) + dot(V, M).dot(V.T)
    
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

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
def residual(a, b):
    """ compute residual (a-b) between measurements containing
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
def ekf_update(ekf, z, landmark):
    ekf.update(z, HJacobian=H_of, Hx=Hx, residual=residual, args=(landmark), hx_args=(landmark))

def get_measurements(mtx, dist, camera_offset):
    barcodes = []
    
    counter = 0
    while (len(barcodes) == 0 and counter < 5): # try to find a landmark 5 times
        # load the input image
        image = undistort(queryImage(), mtx, dist)
        
        # find the barcodes in the image and decode each of the barcodes
        barcodes = pyzbar.decode(image)
        
        counter += 1
    
    if len(barcodes) == 0:
        return None
    else:
        my_return = []      # contains list of arrays [distance, bearing] from car (NOT camera)
        for barcode in barcodes:
            # the barcode data is a bytes object so if we want to draw it on
            # our output image we need to convert it to a string first
            barcodeData = barcode.data.decode("utf-8")
        
            my_coords = barcode.polygon
            barcodeData = barcode.data.decode("utf-8")
            
            landmark_width = None
            
            if barcodeData == "Landmark 1":
                landmark_width = 16.5
            elif barcodeData == "Landmark 2":
                landmark_width = 16.5
            else:
                print ("Incorrect Landmark Detected")
                continue     # should never happen, since only QR codes for 1 and 2 are used
        
            my_focal = 470
            # calculate distance
            if (my_coords[0].y - my_coords[3].y) == 0: # avoid dividing by zero from bad input
                continue
            distance = my_focal*landmark_width / (my_coords[0].y - my_coords[3].y)
            distance = distance*0.01 # convert to meters instead of cm
            # calculate bearing
            my_fov = 2*atan2(640, mtx[0][0])       # calculate field of view in x direction
            x_center = mtx[0][2]
            # average of the x coordinates of the two left corners
            qr_left_edge = (my_coords[0].x + my_coords[3].x)/2
            bearing = (x_center - qr_left_edge) * my_fov/640
            
            # correct for the camera offset
            temp_point = [distance*cos(bearing), distance*sin(bearing)]
            temp_origin = [-camera_offset, 0]
            distance = sqrt((temp_point[0]-temp_origin[0])**2 + (temp_point[1]-temp_origin[1])**2)
            bearing = atan2(temp_point[1]-temp_origin[1], temp_point[0]-temp_origin[0])
            
            if distance < 2 and distance > 0: # sanity check for distance
                if bearing > -60*math.pi/180 and bearing < 60*math.pi/180: # sanity check for bearing
                    my_return.append([distance, bearing, barcodeData])

        return my_return

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
    
    # load calibration measurements for camera from file
    data = np.load('calib.npz')
    my_mtx = data['mtx']
    my_dist = data['dist']
    
    # for plotting course
    xposplot = []
    yposplot = []
    
    # set all known landmarks here (include tag as third element)
    known_landmarks = []
    known_landmarks.append([5*0.3 + 0.09, 5*0.3 + 0.23 - 0.23, "Landmark 1"])
    known_landmarks.append([5*0.3 + 0.09, 5*0.3 + 0.23 - (2*0.3 + 0.23), "Landmark 2"])
    known_landmarks.append([5*0.3 + 0.09, 5*0.3 + 0.23 - (4*0.3 + 0.18), "Landmark 1"])
    known_landmarks.append([0.17, 5*0.3 + 0.23, "Landmark 1"])
    known_landmarks.append([2*0.3 + 0.1, 5*0.3 + 0.23, "Landmark 2"])
    known_landmarks.append([3*0.3 + 0.19, 5*0.3 + 0.23, "Landmark 1"])
    known_landmarks.append([0.0, 0.3, "Landmark 2"])
    known_landmarks.append([0.0, 2*0.3 + 0.22, "Landmark 1"])
    known_landmarks.append([0.0, 4*0.3 + 0.15, "Landmark 2"])
    known_landmarks.append([0.29, 0.0, "Landmark 2"])
    known_landmarks.append([2*0.3 + 0.17, 0.0, "Landmark 1"])
    known_landmarks.append([4*0.3 + 0.07, 0.0, "Landmark 2"])

    # set boundaries here (remember to pad)
    mypad = 0.08
    minx = 0.0 + mypad
    maxx = 5*0.3 + 0.09 - mypad
    miny = 0.0 + mypad
    maxy = 5*0.3 + 0.23 - mypad
    
    # set obstacles here in terms of min and max x and y values (remember to pad)
    obstacles = []
    obstacles.append([0.75 - 0.025 - mypad, 0.75 + 0.025 + mypad, 0.75 - 0.025 - mypad, 0.75 + 0.025 + mypad])

    start_time = time.time()
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
        predicted_position2[0] = predicted_position[0] + cos(predicted_position[2])*(0.18)
        predicted_position2[1] = predicted_position[1] + sin(predicted_position[2])*(0.18)
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
            wanted_time_steps = randint(0, 5)
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
        
        # issue commands to PiCar
        if left_flag:
            if forward_flag:
                forward_left(myspeed)
            else:
                backward_left(myspeed)
        elif right_flag:
            if forward_flag:
                forward_right(myspeed)
            else:
                backward_right(myspeed)
        else:
            if forward_flag:
                forward_straight(myspeed)
            else:
                backward_straight(myspeed)
        
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
        
        # look for landmarks and return a list of distances and bearings from car (NOT camera)
        my_landmarks = get_measurements(my_mtx, my_dist, D+D2)
        if my_landmarks != None:
            # correlate measurements to landmarks
            # entries should look like: [x_coord from ekf.x,
            #                            y_coord from ekf.x,
            #                            measured distance,
            #                            measured bearing]
            seen_landmarks = []
            for lmark in my_landmarks:
                # convert to x and y world coordinates
                myx = ekf.x[0,0] + cos(ekf.x[2,0] + lmark[1])*lmark[0]
                myy = ekf.x[1,0] + sin(ekf.x[2,0] + lmark[1])*lmark[0]
                # iterate through all landmarks in known_landmarks and see if any are a close enough match
                found_flag = False
                for lmark2 in known_landmarks:
                    knownx = lmark2[0]
                    knowny = lmark2[1]
                    if sqrt((knownx-myx)**2 + (knowny-myy)**2) < 0.5 and lmark[2] == lmark2[2]: # if within 40 cm
                        seen_landmarks.append([knownx, knowny, lmark[0], lmark[1]])
                        found_flag = True
                        break
                if not found_flag:
                    # shouldn't happen since all landmarks are known
                    print ("Unknown landmark detected!")
        
            # do ekf_update for each known landmark that is seen
            for lmark in seen_landmarks:
                ekf_update(ekf, np.array([[lmark[2]], [lmark[3]]]), lmark[:2])

        # updating elapsed time
        elapsed_time = time.time() - start_time

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
    time_wanted = 60*5        # how long to run for in seconds
    
    run_picar(start_pose, vel_err, steer_err, range_err, bearing_err, time_wanted)

if __name__ == "__main__":
    main()
