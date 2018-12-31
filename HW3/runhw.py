# import the necessary packages
from pyzbar import pyzbar
import cv2
import requests
import time
import math
import http.client
import numpy as np
import sympy
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

# moves the car to the left at 45 degrees for delta seconds, where 90 is straight forward
def turn_left(delta):
    run_action('fwturn:45')
    run_speed('100')
    run_action('forward_custom')
    #time.sleep(delta)
    #run_action('stop')

# moves the car to the right at 135 degrees for delta seconds, where 90 is straight forward
def turn_right(delta):
    run_action('fwturn:135')
    run_speed('100')
    run_action('forward')
    time.sleep(delta)
    run_action('stop')

# adapted from github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/11-Extended-Kalman-Filters.ipynb
def H_of(x, landmark_pos):
    """ compute Jacobian of H matrix where h(x) computes
        the range and bearing to a landmark for state x """
    
    px = landmark_pos[0]
    py = landmark_pos[1]
    
    myindex = landmark_pos[2] # index in ekf.x of the landmark
    
    hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
    dist = sqrt(hyp)
    
    H = array(
              [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
               [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    
    # put in entries for landmarks
    H = np.append(H, np.zeros((2, x.shape[0]-3)), 1)
    H[0][myindex] = (px - x[0, 0]) / dist
    H[0][myindex+1] = (py - x[1, 0]) / dist
    H[1][myindex] = -(py - x[1, 0]) / hyp
    H[1][myindex+1] = (px - x[0, 0]) / hyp
    
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
        
        # add in zeros for the landmarks entries
        dx = np.append(dx, np.zeros((x.shape[0]-3, 1)), 0)
        
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

def landmark_update(ekf, landmark):
    x_coord = landmark[0]
    y_coord = landmark[1]
    
    # update some internals
    ekf._I = np.eye(ekf._I.shape[0]+2)
    
    # update the state variable
    ekf.x = np.append(ekf.x, np.array([[x_coord, y_coord]]).T, 0)
    
    # update the covariance variable
    dim = ekf.P.shape[0]
    temp = np.zeros((dim+2, dim+2))
    temp[:dim,:dim] = ekf.P
    temp[dim,dim] = ekf.P[0,0]
    temp[dim+1,dim+1] = ekf.P[1,1]
    ekf.P = temp
    
    # update the Jacobian of F (note this is a Sympy Matrix)
    dim = ekf.F_j.shape[0]
    temp2 = sympy.eye(dim+2)
    temp2[:dim,:dim] = ekf.F_j
    ekf.F_j = temp2
    
    # update the Jacobian of V (note this is a Sympy Matrix)
    dim = ekf.V_j.shape[0]
    temp3 = sympy.zeros(dim+2,2)
    temp3[:dim,:2] = ekf.V_j
    ekf.V_j = temp3

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

# runs in a circle going counter-clockwise
# if figure_eight_flag is set, then uses half the steps going counter-clockwise, followed
# by half the steps going clockwise
def run_circle(std_vel, std_steer, std_range, std_bearing, num_steps, figure_eight_flag = False):
    delta = 0.1
    D = 0.15     # i measured approx 15cm between back wheels and front wheels
    D2 = 0.08    # i measured approx 8cm between camera and front wheels
    vel = 0.6       # at speed of 100 out of 100
    ekf = RobotEKF(delta, wheelbase=D, std_vel=std_vel, std_steer=std_steer)
    ekf.x = array([[0, 0, 0]]).T    # x, y, orientation
    ekf.P = np.diag([0, 0, 0])      # no uncertainty at very start
    ekf.R = np.diag([std_range**2, std_bearing**2])
    
    my_steer = 12*math.pi/180   # although the command sent to the PiCar is for 45 degrees, it seems
                                # to be closer this many degrees (away from forward) in reality
    
    # steering command (velocity, steering angle in radians)
    u = array([vel, my_steer])
    left_flag = True
    
    # load calibration measurements for camera from file
    data = np.load('calib.npz')
    my_mtx = data['mtx']
    my_dist = data['dist']
    
    xposplot = []
    yposplot = []
    
    time_added = []
    
    tag_dict = {}
    
    for i in range(num_steps+1):
        xposplot.append(ekf.x[0,0])
        yposplot.append(ekf.x[1,0])
        
        # initial landmark adding
        if i == 0:
            my_landmarks = get_measurements(my_mtx, my_dist, D+D2)
            if my_landmarks != None:
                # add in landmarks
                tempindex = 3
                for lmark in my_landmarks:
                    tag_dict[tempindex] = lmark[2]
                    time_added.append(i)
                    landmark_update(ekf, lmark)
                    tempindex += 1
            continue
        
        # for doing figure eight
        if figure_eight_flag and i == num_steps//2:
            u = array([vel, -my_steer])
            left_flag = False

        # issue commands to PiCar
        if left_flag:
            turn_left(delta)
        else:
            turn_right(delta)

        # do internal pose prediction
        ekf.predict(u=u)
        
        # look for landmarks and return a list of distances and bearings from car (NOT camera)
        my_landmarks = get_measurements(my_mtx, my_dist, D+D2)
        if my_landmarks == None:
            continue
        # separate landmarks into old and new by comparing distances to known landmarks
        # old should look like: [x_coord from ekf.x,
        #                       y_coord from ekf.x,
        #                       starting index in ekf.x,
        #                       measured distance,
        #                       measured bearing]
        # new should look like: [x_coord from measurements,
        #                       y_coord from measurements]
        known_landmarks = []
        unknown_landmarks = []
        for lmark in my_landmarks:
            # convert to x and y world coordinates
            myx = ekf.x[0,0] + cos(ekf.x[2,0] + lmark[1])*lmark[0]
            myy = ekf.x[1,0] + sin(ekf.x[2,0] + lmark[1])*lmark[0]
            # iterate through all landmarks in ekf.x and see if any are a close enough match
            found_flag = False
            for j in range(3, ekf.x.shape[0], 2):
                knownx = ekf.x[j,0]
                knowny = ekf.x[j+1,0]
                if sqrt((knownx-myx)**2 + (knowny-myy)**2) < 0.5 and lmark[2] == tag_dict[j]: # if within 40 cm
                    known_landmarks.append([knownx, knowny, j, lmark[0], lmark[1]])
                    found_flag = True
                    break
            if not found_flag:
                unknown_landmarks.append([myx, myy, lmark[2]])
        
        # do ekf_update for each known landmark that is seen
        for lmark in known_landmarks:
            ekf_update(ekf, np.array([[lmark[3]], [lmark[4]]]), lmark[:3])

        # add in new landmarks
        tempindex = ekf.x.shape[0]
        for lmark in unknown_landmarks:
            time_added.append(i)
            tag_dict[tempindex] = lmark[2]
            landmark_update(ekf, lmark)
            tempindex += 1

    print (ekf.x)
    xplot = []
    yplot = []
    for i in range(3, ekf.x.shape[0], 2):
        xplot.append(ekf.x[i,0])
        yplot.append(ekf.x[i+1,0])
    plt.plot(xposplot, yposplot, color ='r')
    plt.scatter(xplot, yplot)
    for i, txt in enumerate(time_added):
        plt.annotate(txt, (xplot[i], yplot[i]))
    plt.show()

def main():
    vel_err = 0.03          # error in terms of std dev for velocity
    steer_err = 0.05        # error in terms of std dev for steering angle
    range_err = 0.06        # error in terms of std dev for range measurement
    bearing_err = 0.06      # error in terms of std dev for bearing measurement
    steps_needed = 80     # number of steps needed for 1 full circle
    # 1 circle run
    #run_circle(vel_err, steer_err, range_err, bearing_err, steps_needed)
    # 2 circle run
    run_circle(vel_err, steer_err, range_err, bearing_err, steps_needed*2)
    # figure eight run
    #run_circle(vel_err, steer_err, range_err, bearing_err, steps_needed*2, figure_eight_flag = True)

if __name__ == "__main__":
    main()
