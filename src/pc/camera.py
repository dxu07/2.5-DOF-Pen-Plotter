'''!
    @file       camera.py
    
    @brief      Program that runs on the PC to find and shoot at red targets. 
    
    @details    This program utilizes opencv to help detect red objects from the camera. 
                Once run, it will constantly scan for red object and calculate the distance 
                between the center of the screen to the red object. Then by knowing the
                field of view of the camera, a direct polar and azimuthal coordinate can be sent over. 
                The program will only begin scanning for the red object when it detects that there is no motion. 
                This allows for it to recalculate a new angle once the stepper motors have moved to a new point.
                The instruction are sent through serial to the MCU. 
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       02/23/2022
'''
import cv2
import imutils
import numpy as np
import serial
import sys

# Create Window
cv2.namedWindow("scope")

#Capture Film
vc = cv2.VideoCapture(0)
vc.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
vc.set(cv2.CAP_PROP_FPS, 30)
vc.set(cv2.CAP_PROP_FOCUS, 10)

# Frame Size
wdth = 900
higt = wdth / 1920 * 1080

# Setting Variables to be used later 
x_avg = 0
y_avg = 0

polar = 0
azimuth = 0

x_frame = []
y_frame = []

on_target = 0

new_angle = False

fired = False

i = 1

x_angle = 75
y_angle = 47
x_correction = 2
y_correction = 2

kernal = np.ones((7, 7), "uint8")

lastFrame = None

red_lower = np.array([136, 135, 130], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)

if vc.isOpened():  # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

flag = 1

while rval:

    frame = imutils.resize(frame, width=wdth)

    # convert to hsv colorspace
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Gray frames for motion detection comparison
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # Sets previous frame for motion detection
    if lastFrame is None:
        lastFrame = gray
        continue

    # Motion Detection
    # compute the absolute difference between the current frame and last frame
    frameDelta = cv2.absdiff(lastFrame, gray)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)

    # Create contours of difference between frames
    motion_cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    motion_cnts = imutils.grab_contours(motion_cnts)

    # Checks to see if any contours exist. If no contours exist, there is no motion and a new angle should be determined
    if len(motion_cnts) == 0:
        new_angle = True

    # Sets current frame to last frame to check for motion next loop
    lastFrame = gray

    # Checking for the color red
    # https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv
    mask = cv2.inRange(hsv, red_lower, red_upper)
    mask = cv2.dilate(mask, kernal)

    # Big Crosshair in center
    cv2.line(frame, (int(wdth / 2), int(higt / 2 - 20)), (int(wdth / 2), int(higt / 2 + 20)), (0, 255, 0), 2)
    cv2.line(frame, (int(wdth / 2 - 20), int(higt / 2)), (int(wdth / 2 + 20), int(higt / 2)), (0, 255, 0), 2)

    # Find contours from the mask, aka check if there is any red
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # IF there is red AND there is no motion AND the nerf gun has not fired yet
    if len(contours) > 0 and new_angle and not fired:

        cnts = np.concatenate(np.concatenate(contours))

        x = [x[0] for x in cnts]
        y = [y[1] for y in cnts]

        x_frame.extend(x)
        y_frame.extend(y)

        # Update angle every 50 loops
        if i % 50 == 0:
            # Average x,y position of red
            x_avg = sum(x_frame)/len(x_frame)
            y_avg = sum(y_frame)/len(x_frame)
            x_frame = []
            y_frame = []

            # Distance from red blob to middle
            x_dist = (x_avg - wdth/2)
            y_dist = (y_avg - higt/2)
            dist = np.sqrt(np.power(x_dist, 2) + np.power(y_dist, 2))

            # Determine Angles
            polar += (x_avg-(wdth/2))/(wdth/2)*x_angle/2
            azimuth += (y_avg-(higt/2))/(higt/2)*y_angle/2

            # Check to fire
            if dist <= 10:
                on_target += 1
                if on_target >= 10:
                    fire = 1
                    # polar -= x_correction
                    # azimuth -= y_correction
                    fired = True
                    on_target = 0

                else:
                    fire = 0
                    new_angle = False

            else:
                fire = 0
                on_target = 0
                new_angle = False

            # Create instructions to send to MCU
            instr = 'd:' + str(polar*-1) + ',' + str(azimuth*-1) + ',' + str(fire)
            print(instr)
            with serial.Serial(sys.argv[1], 115200, 8, 'N', 1) as ser:
                ser.write(instr.encode())

            # Clear INSTR
            polar = 0
            azimuth = 0
            x_avg = 0
            y_avg = 0

        # print((x_avg, y_avg))

        # Small Crosshair
        cv2.line(frame, (int(x_avg), int(y_avg+10)), (int(x_avg), int(y_avg-10)), (0, 255, 0), 2)
        cv2.line(frame, (int(x_avg-10), int(y_avg)), (int(x_avg+10), int(y_avg)), (0, 255, 0), 2)

        i += 1

    frame = cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)

    cv2.imshow("scope", frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)

    if key == 27:  # exit on ESC
        break

vc.release()
cv2.destroyWindow("scope")
