'''!
    @file       ProcessHPGL.py
    
    @brief      This program takes in an HPGL file and converts it into useable angles for our stepper motors
    
    @details    This program takes in an HPGl file and reads it. The main task 
                in this file is to receive and read a command from uart, 
                instructing the program on which file it should parse. Once it 
                parses the file, it begins filtering and file to remove excess noise generated from HPGL creation. 
                Once the noise is removed, it begins to process and interpolate coordinates necessary for our drawing. 
                The interpolated coordinates are then put through the Newton-Raphson iterater, and angular coordinates 
                are sent to a different task. 
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''


import micropython, pyb
from pyb import UART

try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
import os

# constants
#   todo: calibrate?
d = 108
l = 12
h = 0


def sec(t):
    '''!@brief      This function finds the secant of an angle. 

        @param      t is the angle
        @return     sec(t)
    '''
    return 1 / np.cos(t)


def sec2(t):
    '''!@brief      This function find the secant squared of a value
        @param      t is the angle
        @return     sec2 which is the secant squared
    '''
    return pow(sec(t), 2)


def g(x, theta):
    '''!@brief      This function finds the theta values based off of the x,y coordinates. 
        @details    This follows the Newton-Raphson method of determining theta based off of x,y 
                    coordinates through equations that we established in the kinematics calculations
        @param      x is the tuple of x,y coordinates
        @param      theta is the angle used for the guess
        @return     gm which the angle. 
    '''
    gm = np.zeros(2)

    # x-dimension
    gm[0] = x[0] + (d + h * np.sin(theta[1])) * np.tan(theta[0])
    # y-dimension
    gm[1] = x[1] - h * np.cos(theta[1]) - np.tan(theta[1]) * (d + h * np.sin(theta[1]))

    return gm


def dg_theta(theta):
    '''!@brief      This function finds the derivative of theta values based off of the x,y coordinates. 
        @details    This follows the Newton-Raphson method of determining derivative of the theta value equations
                    based off of x,y coordinates through equations that we established in the kinematics calculation
        @param      theta is the angle used for the guess
        @return     dgm the derivative of the angle guesses
    '''
    dgm = np.zeros((2, 2))

    # x-dimension
    dgm[0][0] = (d + h * np.sin(theta[1])) * sec2(theta[0])
    dgm[0][1] = h * np.cos(theta[1]) * np.tan(theta[0])
    # y-dimension
    dgm[1][0] = 0
    dgm[1][1] = -sec2(theta[1]) * (d + h * np.sin(theta[1]))

    return dgm


def newton_raphson(fcn, jacobian, guess, thresh):
    '''!@brief      This function does the Newton Raphson iterations
        @details    This function finds the roots of the equations using the Newton-Raphson 
                    method to find the desired angular coordinates.
        @param      fcn lambda function that is used to iterate through angles.
        @param      jacobian Jacobian matric derived from kinematics calculations
        @param      guess Guess from the previous iterations
        @param      thresh Determines how close to target coordinate is acceptable
        @return     result The result of the iteration, also the angular data. 
    '''
    result = guess - np.dot(np.linalg.inv(jacobian(guess)), fcn(guess))

    while any(abs(_) > thresh for _ in fcn(result)):
        result = result - np.dot(np.linalg.inv(jacobian(result)), fcn(result))

    return result


def compute_steps(xy_des, queues):
    '''!@brief      This function utilizes the newton_raphson function. 
        @details    This function utilizes the newton_raphson function to find 
                    angular data corresponding the desired x,y coords. It then 
                    puts it into the Queue.
        @param      xy_des desired x,y coordinates for newton_raphson to go through
        @param      queues the shared queues that we put our data into. 
    '''
    # unpack queues
    p, a, f = queues

    last_guess = [0, 0]
    for i in range(len(xy_des)):
        # wait until positioning task un-blocks
        while p.any() or a.any() or f.any():
            yield
        # compute desired position
        last_guess = newton_raphson(lambda theta: g(xy_des[i], theta), dg_theta, last_guess, 1e-3)
        print('moving to: [' + str((180 * last_guess[0]) / np.pi) + ', ' + str((180 * last_guess[1]) / np.pi) + ']')

        # update target point
        # todo: put without blocking
        p.clear()
        p.put((180 * last_guess[0]) / np.pi)
        a.put((180 * last_guess[1]) / np.pi)
        # we only move to places we are going to fire
        f.put(1)

        yield


def coords(line, pen_type):
    '''!@brief      This function takes the hpgl lines and determines the x,y coordinates.
        @details    This function reads the PU, PD of the hpgl lines and determines if they
                    represent x or y coordinates. 
        @param      Line is the line read from the hpgl file
        @pentype    Is the PU or PD instructions 
                    
        @return     list of x,y coordinates and whether the coordinate is PU or PD. 
    '''
    line = line[2:].strip().split(',')
    x_coords = [(int(j) / 1000) * 5 for i, j in enumerate(line) if i % 2 == 0]
    y_coords = [(int(j) / 1000) * 5 for i, j in enumerate(line) if i % 2 == 1]
    pen = [pen_type] * len(x_coords)
    return list(zip(pen, x_coords, y_coords))


def filter_hpgl(cart_coords):
    '''!@brief      This function removes excess noise from the HPGL files.
        @details    HPGL files inherintly come with a lot of noise, so this function 
                    removes points that are too close together and also any unecessary instruction.,
        @param      cart_coords is the list of unfiltered cartesian coordinates. 
        @return     filterd_coords is the list of filterd cartesian coordinates
    '''
    filtered_coords = []
    for i in range(0, (len(cart_coords) - 1)):
        filtered_coords.append(cart_coords[i])
        if i >= 1:
            l = cart_coords[i - 1][0]
            r = cart_coords[i + 1][0]
            xdiff = abs(filtered_coords[-1][1] - filtered_coords[-2][1])
            ydiff = abs(filtered_coords[-1][2] - filtered_coords[-2][2])
            if filtered_coords[-2][0] == cart_coords[i][0] == 0:
                filtered_coords = filtered_coords[:-2]
                filtered_coords.append(cart_coords[i])
            elif l == 0 and r == 0:
                filtered_coords = filtered_coords[:-1]
            elif cart_coords[i][0] == 1 and filtered_coords[-2][0] != 0:
                if xdiff <= ((300 * 5) / 1000) and ydiff <= ((300 * 5) / 1000):
                # if (xdiff * ydiff) <= ((300 * 5) / 1000):
                    filtered_coords = filtered_coords[:-1]
    return filtered_coords


def draw(cart_coords, queues):
    '''!@brief      This function interpolates cartesian coordinates between points. 
        @details    This function find the number of steps necessary between two PD points. 
                    It has a set distance between points, and interpolates so that the 
                    distance between points is consistent. 
        @param      cart_coords is the list of filtered cart_coords that needs to be interpolated.
        @param      queues is the shared queues that we use. 
    '''
    print('in draw')
    for i in range(0, (len(cart_coords) - 1)):
        if cart_coords[i][0] == cart_coords[i + 1][0] == 1:
            xy1 = (cart_coords[i][1], cart_coords[i][2])
            xy2 = (cart_coords[i + 1][1], cart_coords[i + 1][2])
            NUM_STEPS = round(np.sqrt(pow(xy2[0] - xy1[0], 2) + pow(xy2[1] - xy1[1], 2)) / 2.5)
            print('found line: ' + str(NUM_STEPS) + ' from ' + str(xy1) + ' to ' + str(xy2))
            x_des = np.linspace(xy1[0], xy2[0], num=max(NUM_STEPS,2), endpoint=False)
            y_des = np.linspace(xy1[1] + 12, xy2[1] + 12, num=max(NUM_STEPS,2), endpoint=False)
            # xy_des = np.transpose([x_des,y_des])
            xy_des = np.array([x_des, y_des]).transpose()

            yield from compute_steps(xy_des, queues)
        yield


def task_process_hpgl(fname, queues):
    '''!@brief      This is the task function that puts all the hpgl functions together.
        @details    This function goes through and does both the Newton-Raphson and the 
                    hpgl/coordinates stuff. 
        @param      fname is the name of the function associated with task_process_hpgl
        @param      queues is the queue that is shared between tasks. 
    '''
    while 1:
        if fname.any():
            # get filename from queue
            filename = []
            while fname.any():
                filename.append(fname.get())
            filename = bytearray(filename).decode()

            # read hpgl from file
            with open('hpgl/' + filename, 'r') as file:
                line = file.readline()

            # parse into commands
            commands = line.strip().split(';')

            # parse commands into points
            cart_coords = []
            for command in commands:
                if len(command) > 2:
                    if 'PU' in command:
                        cart_coords.extend(coords(command, 0))
                    elif 'PD' in command:
                        cart_coords.extend(coords(command, 1))

            print(len(cart_coords))

            # filter coords
            #   todo: yield
            cart_coords = filter_hpgl(cart_coords)

            print(len(cart_coords))

            # generate positioning commands
            yield from draw(cart_coords, queues)
        yield
