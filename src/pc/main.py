'''!
    @file       main.py
    
    @brief      Program runs on the PC and sends user input instructions to the MCU
    
    @details    This program sends commands from the PC to the MCU through serial
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''
import serial
import sys


def main():
    '''!@brief      This function writes commands to serial
    '''
    with serial.Serial(sys.argv[1], 115200, 8, 'N', 1) as ser:
        while 1:
            s = input("enter command to send: ")
            ser.write(s.encode())
            # if ser.inWaiting():
            #     print(ser.readline().decode())
            #l = ser.readline()
            #print(l.decode())


if __name__ == "__main__":
    main()
