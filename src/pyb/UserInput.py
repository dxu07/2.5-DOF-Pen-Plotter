'''!
    @file       UserInput.py
    
    @brief      This program reads in commands from uart
    
    @details    This program reads commands from uart and parses them. The commands 
                are sent in from the PC and then interpreted. The instructions 
                are then sent to the other tasks accordingly.
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''
import pyb
import os

MAX_FILENAME = 100


def task_user_input(queues, filename, paused, stopped):
    '''!@brief      This function reads the user input
        @details    This function reads the user input from uart and them provides necessary 
                    information to the other tasks. This includes direct commands, drawing commands, 
                    stopping, and resuming. 
        @param      queues is the queue of positional data
        @param      filename is the name of the hpgl file to be drawn
        @param      paused is a boolean to check if the drawing task needs to pause
        @param      stopped is the boolean to check if the stepper motors need to stop.
    '''
    uart = pyb.UART(2, 115200, bits=8, parity=None, stop=1)

    while 1:
        if uart.any():
            pyb.delay(100)
            # read line
            line = uart.readline().decode().strip()
            if ':' in line:
                line = line.split(':')
                # parse command
                command = line[0]
                # parse args
                #   todo: split on newline?
                args = line[1].split(',')
            else:
                command = line

            # direct command (d:0.0,0.0,0)
            if command == 'd':
                # check args
                if len(args) != 3:
                    uart.write(bytearray('x: d - bad # of args'.encode()))
                    continue
                # attempt to parse points
                try:
                    p, a, f = float(args[0]), float(args[1]), int(args[2])
                except ValueError:
                    uart.write(bytearray('x: d - bad value of args'.encode()))
                    continue
                # update queues if move is completed
                pp, aa, ff = queues
                if not pp.any() and not aa.any() and not ff.any():
                    pp.put(p)
                    aa.put(a)
                    ff.put(f)
                else:
                    # todo: send error (position not reached)
                    pass

            # draw file command (f:xyz.hpgl)
            elif command == 'f':
                # check for filename
                if not args:
                    uart.write(bytearray('x: f - no file specified'.encode()))
                    continue
                # check for length
                if len(args[0]) > MAX_FILENAME:
                    uart.write(bytearray('x: f - filename too long'.encode()))
                    continue
                # check if file exists
                try:
                    os.stat('hpgl/' + args[0])
                except OSError:
                    uart.write(bytearray('x: f - file does not exist'.encode()))
                    continue
                # update share
                filename.clear()
                for c in args[0].encode():
                    filename.put(c)
                print(filename)

            # upload file command (u:abc.hpgl - data - '\n')
            elif command == 'u':
                # check for filename
                if not args:
                    uart.write(bytearray('x: u - no file specified'.encode()))
                    continue
                # check for length
                if len(args[0]) > MAX_FILENAME:
                    uart.write(bytearray('x: u - filename too long'.encode()))
                    continue
                # read bytes until newline
                with open('hpgl/' + args[0], 'w') as file:
                    c = uart.readchar()
                    while c != '\n':
                        # write to file
                        file.write(chr(c))
                        c = uart.readchar()

            # pause command (p)
            elif command == 'p':
                # pause scheduling of hpgl and positioning task
                paused.put(1)
            # resume command (r)
            elif command == 'r':
                # resume scheduling of hpgl and positioning task
                paused.put(0)
            # stop command (e)
            elif command == 'e':
                # clear all shares, triggering e-stop
                #for queue in queues:
                #    queue.clear()
                #filename.clear()
                stopped.put(1)
                pass
            # home command (h)
            elif command == 'h':
                # goto 0.0,0.0,0, pre-empting
                for queue in queues:
                    queue.put(0)
            # calibrate command (c)
            #   todo: have positioning task re-calibrate
            elif command == 'c':
                pass
            # unknown command
            else:
                uart.write(bytearray('x: unknown command'.encode()))
        yield
