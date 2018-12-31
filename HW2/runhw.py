# import the necessary packages
from pyzbar import pyzbar
import numpy as np
import cv2
import requests
import time
import math
import http.client

# set the correct host and port here
HOST = '192.168.1.76'
PORT = '8000'
IMG_PORT = '8080'

BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

max_speed = 50 # at speed of 100, i measured approx 50cm/sec
scale = 0.5 # scaling waypoints so that it doesn't take so much physical space to run
L = 15 # i measured approx 15cm between back wheels and front wheels
L2 = 8 # i measured approx 8cm between camera and front wheels

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

def get_distance(mtx, dist):
    barcodes = []
    
    counter = 0
    while (len(barcodes) == 0 and counter < 20):
        # load the input image
        image = undistort(queryImage(), mtx, dist)
        
        # find the barcodes in the image and decode each of the barcodes
        barcodes = pyzbar.decode(image)
    
        counter += 1
    
    if len(barcodes) > 0:
        # just use the first barcode
        barcode = barcodes[0]
        
        # the barcode data is a bytes object so if we want to draw it on
        # our output image we need to convert it to a string first
        barcodeData = barcode.data.decode("utf-8")
        
        my_coords = barcode.polygon
        my_coords2 = []
        for my_point in my_coords:
            my_coords2.append((my_point.x, my_point.y))
        barcodeData = barcode.data.decode("utf-8")
        
        landmark_width = None
        
        if barcodeData == "Landmark 1":
            landmark_width = 17.5
        elif barcodeData == "Landmark 2":
            landmark_width = 17.8
        elif barcodeData == "Landmark 3":
            landmark_width = 17.0
        elif barcodeData == "Goal":
            landmark_width = 17.2

        my_focal = 470
        distance = my_focal*landmark_width / (my_coords[0].y - my_coords[3].y)
        return distance

# load calibration measurements for camera from file
data = np.load('calib.npz')
my_mtx = data['mtx']
my_dist = data['dist']

def left_forward(mydelta, speed):
    run_action('fwturn:0')
    run_speed(str(speed))
    run_action('forward')
    time.sleep(mydelta)
    run_action('stop')

def right_backward(mydelta, speed):
    run_action('fwturn:180')
    run_speed(str(speed))
    run_action('backward')
    time.sleep(mydelta)
    run_action('stop')

def forward(mydelta, speed):
    run_action('fwturn:90')
    run_speed(str(speed))
    run_action('forward')
    time.sleep(mydelta)
    run_action('stop')

def turn_left():
    myspeed = 50
    left_forward(0.5, myspeed)
    right_backward(0.5, myspeed)
    left_forward(0.5, myspeed)
    right_backward(0.5, myspeed)
    left_forward(0.5, myspeed)
    right_backward(0.5, myspeed)
    left_forward(0.5, myspeed)
    right_backward(0.5, myspeed)
    left_forward(0.4, myspeed)
    right_backward(0.4, myspeed)
    #left_forward(0.1, myspeed)
    #right_backward(0.1, myspeed)
    forward(0.2, 100)


# move forward until distance from qr marker 1 is 70 cm
while (get_distance(my_mtx, my_dist) > 70 - (L + L2)):
    # make car run for 0.1 seconds
    forward(0.1, 50)
    

# turn left
turn_left()

# move forward until distance from qr marker 2 is 40 cm
while (get_distance(my_mtx, my_dist) > 78  - (L + L2)):
    # make car run for 0.1 seconds
    forward(0.1, 50)

# turn left
turn_left()

# move forward until distance from qr marker 3 is 70 cm
while (get_distance(my_mtx, my_dist) > 70  - (L + L2)):
    # make car run for 0.1 seconds
    forward(0.1, 50)

# turn left
turn_left()

# move forward until distance from qr marker goal is 60 cm
while (get_distance(my_mtx, my_dist) > 60  - (L + L2)):
    # make car run for 0.1 seconds
    forward(0.1, 50)

# turn left
turn_left()
