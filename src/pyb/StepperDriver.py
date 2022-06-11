'''!
    @file       StepperDriver.py
    
    @brief      This is a class program that interfaces with both the TMC2208 and the TMC4210. 
    
    @details    This program acts as the bridge between the microcontroll and the stepper motor drivers. 
                This allows the mcu to send and read instructions from the stepper motor drivers. It has 
                the capabilities of enabling the stepper motor drivers and handles the IC for the TMC4210.
                It is capable of changing the motor operating modes, set velocities, and positions. 
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''
import pyb

# stepper motor registers
X_TARGET = 0x00
X_ACTUAL = 0x01
V_MIN = 0x02
V_MAX = 0x03
V_TARGET = 0x04
V_ACTUAL = 0x05
A_MAX = 0x06
A_ACTUAL = 0x07
PMUL_PDIV = 0x09
REFCONF_RAMPMODE = 0x0A
INTERRUPT_MASK_FLAGS = 0x0B
PULSE_RAMP_DIV = 0x0C
DX_REF_TOLERANCE = 0x0D
X_LATCHED = 0x0E
USTEP_COUNT_4210 = 0x0F

# global parameter registers
IF_CONFIGURATION_4210 = 0x34
POS_COMP_4210 = 0x35
POS_COMP_INT_4210 = 0x36
POWER_DOWN = 0x38
TYPE_VERSION = 0x39
REFERENCE_SWITCHES = 0x3E
GLOBAL_PARAMETERS = 0x3F

# ramp modes
RAMP_MODE = 0b00
SOFT_MODE = 0b01
VELOCITY_MODE = 0b10
HOLD_MODE = 0b11


class TMC2208:
    '''!@brief      A TMC2208 driver class.
        @details    Objects of this class can be used to configure the TMC2208.
                    This involves enables and disabling the stepper motors. 
    '''
    def __init__(self, en):
        '''!@brief      Initializes the TMC2208 
            @param      en is the pin associated with enabling and disabling the driver.
         
        '''
        self.en = en
        self.disable()

    def enable(self):
        '''!@brief      Enables to stepper motors by setting the enable pin low
         
        '''
        self.en.value(0)

    def disable(self):
        '''!@brief      Disables the stepper motors by setting the enable pin high
         
        '''
        self.en.value(1)


def sign_extend(val, width):
    '''!@brief      Handles sign extension the python way
        @param      val the value that needs to be sign extended
        @param      width how long is value is
     
    '''
    if val & (0x1 << (width - 1)):
        val -= 0x1 << width
    return val


# todo: limit switches/calibration
class TMC4210:
    '''!@brief      A TMC4210 class.
        @details    Objects of this class can be used to configure the TMC4210.
                    This involves reading and writing to its registries.  
    '''
    def __init__(self, spi_bus, cs, v_max, a_max):
        '''!@brief      Initializes the TMC4210 
            @param      spi_bus is the spi object created to interface with this peripheral
            @param      cs is the chip select unique to the peripheral
            @param      v_max is the maximum velocity used for stepper motor speed control, in steps per unit time
            @param      a_max is the a maximum acceleration used for stepper motor speed control, in steps per (unit time)^2
         
        '''
        self.spi_bus = spi_bus
        self.cs = cs
        self.cs.value(1)

        # todo: base dividers on 20MHz clock
        #   velocity = steps / unit time <-> PULSE_DIV
        #   accel = (steps / unit time ^ 2) / 256 <-> RAMP_DIV
        #       range 0 - 2047
        # todo: calculate programmatically from desired RPM
        RAMP_DIV = 10
        PULSE_DIV = 10

        # enable Step/Dir interface
        self.rw_value(0, IF_CONFIGURATION_4210, 0x20)

        # configure clock pre-dividers
        self.rw_value(0, PULSE_RAMP_DIV, (PULSE_DIV & 0x0F) << 12 | (RAMP_DIV & 0x0F) << 8)

        # set velocity range
        self.rw_value(0, V_MIN, 1)
        self.rw_value(0, V_MAX, v_max)

        # optimized calculation of PMUL and PDIV
        p = a_max / (128 * pow(2, RAMP_DIV - PULSE_DIV))
        for PDIV in range(0, 14):
            PMUL = 0.99 * p * pow(2, 3) * pow(2, PDIV)
            if 128 <= PMUL <= 255:
                print('found val pmul: ' + str(PMUL) + 'pdiv: ' + str(PDIV))
                break
        # set acceleration and proportionality factor
        self.rw_value(0, A_MAX, a_max)
        self.rw_value(0, PMUL_PDIV, 0x8000 | (int(PMUL) & 0x7F) << 8 | (int(PDIV) & 0x0F))

        # defaults to ramp mode
        self.set_mode(RAMP_MODE)
        # setup both reference switches
        s, d = self.rw_value(1, GLOBAL_PARAMETERS, 0)
        d |= 0x200000
        self.rw_value(0, GLOBAL_PARAMETERS, d)

    def rw_value(self, rw, reg, data):
        '''!@brief      Handles reading and writing to registries
            @param      rw is the instruction on whether to read or write
            @param      reg is the registry to be accessed
            @param      data is the data to be written to registry
            @return     status returns the status of the registry
            @return     data is the data read from the registry
         
        '''
        # data should always be zero for read (rw == 1)
        if rw:
            data = 0

        buff = bytearray(4)

        buff[0] = (reg & 0x3F) << 1 | rw & 0x01
        buff[1] = (data & 0xFF0000) >> 16
        buff[2] = (data & 0x00FF00) >> 8
        buff[3] = data & 0x0000FF

        # print(''.join('{:02x}'.format(x) for x in buff))

        self.cs.value(0)
        self.spi_bus.send_recv(buff, buff)
        self.cs.value(1)

        # print(''.join('{:02x}'.format(x) for x in buff))

        status = buff[0]
        data = (buff[1] << 16) | (buff[2] << 8) | buff[3]

        return status, data

    def set_mode(self, mode):
        '''!@brief     Changes the mode in which the stepper motors are operating in
            @param     mode is the mode to be changed to. 
         
        '''
        # read existing shared register
        s, d = self.rw_value(1, REFCONF_RAMPMODE, 0)
        # mask and set RAMP_MODE bits
        d &= ~(0b11)
        d |= (mode & 0b11)
        # write back to shared register
        self.rw_value(0, REFCONF_RAMPMODE, d)

    def set_target_position(self, pos):
        '''!@brief     Sets the target Position
            @param     pos the position to be set
         
        '''
        self.rw_value(0, X_TARGET, pos)

    def get_target_position(self):
        '''!@brief      Reads the target position
            @return     d returns the data at the target position
         
        '''
        s, d = self.rw_value(1, X_TARGET, 0)
        return d

    def get_actual_position(self):
        '''!@brief      Reads the actual position
            @return     d returns what the actual position is
         
        '''
        s, d = self.rw_value(1, X_ACTUAL, 0)
        return sign_extend(d, 24)

    def is_target_reached(self):
        '''!@brief      checks to see if motor has reached the target position
            @return     boolean of whether or not it has been met
         
        '''
        # read dummy register
        s, d = self.rw_value(1, TYPE_VERSION, 0)
        # return xEQt1 bit
        return s & 0x01

    def set_target_velocity(self, v_target):
        '''!@brief     Sets the target velocity
            @param     v_target the velocity to be set
         
        '''
        self.rw_value(0, V_TARGET, v_target)

    def get_target_velocity(self):
        '''!@brief      Reads the target velocity
            @return     d returns the data at the target position
         
        '''
        s, d = self.rw_value(1, V_TARGET, 0)
        return sign_extend(d, 12)

    def get_actual_velocity(self):
        '''!@brief      Reads the actual velocity
            @return     d returns what the actual velocity is
         
        '''
        s, d = self.rw_value(1, V_ACTUAL, 0)
        return sign_extend(d, 12)


def map_range(v, x1, x2, y1, y2):
    '''!@brief      maps the range
        @return     returns the mapped range
     
    '''
    return int(y1 + ((v-x1)*(y2-y1))/(x2-x1))


class StepperDriver:
    '''!@brief      StepperDriver class that interfaces both the TMC4210 and the TMC2208 together.
        @details    This class creates objects that work with both TMC4210 and the TMC2208. 
                    This allows for separate objects that can control the motors independently.
    '''
    def __init__(self, spi_bus, cs, en):
        '''!@brief      Initializes the StepperDriver
            @param      spi_bus is the spi object created to interface with this peripheral
            @param      cs is the chip select unique to the peripheral
            @param      en is the enable pin for the TMC2208
         
        '''
        
        # configurable vmax and amax

        self.controller = TMC4210(spi_bus, pyb.Pin(cs, pyb.Pin.OUT_PP), 1288, 512)

        self.driver = TMC2208(pyb.Pin(en, pyb.Pin.OUT_PP))

        # set by the stepper motor
        self.degrees_per_step = 1.8
        # set by the stepper control board
        self.micro_steps = 8
        # how many steps in a single revolution
        self.steps_per_rotation = (360 * self.micro_steps) / self.degrees_per_step

        # map over entire angular range
        self.min_angle = 0
        self.max_angle = 360

        # map over entire stepper range
        self.min_step = 0
        self.max_step = self.steps_per_rotation

    def set_range(self, a1, a2, s1, s2):
        '''!@brief      sets the range at which the stepper motor can operate
            @param      a1 minimum angle
            @param      a2 maximum angle
            @param      s1 minumum step
            @param      s2 maximum step
         
        '''
        self.min_angle = a1
        self.max_angle = a2
        self.min_step = s1
        self.max_step = s2

    # todo: yield while waiting for position to latch
    def calibrate(self, a1, a2):
        '''!@brief      Calibrates the stepper motors based on the calibration instruction in the TMC4210 data manual. 
            @param      a1 angle associated with one of the limit switches
            @param      a2 angle associated with the other limit switch.
         
        '''
        # set to velocity mode
        self.controller.set_mode(VELOCITY_MODE)
        # read ramp mode
        s, d = self.controller.rw_value(1, REFCONF_RAMPMODE, 0)

        # setup right switch for homing
        self.controller.rw_value(0, REFCONF_RAMPMODE, d | 0x800)
        # init latching mechanism
        self.controller.rw_value(0, X_LATCHED, 0)
        # move in positive direction until limit is hit
        self.controller.set_target_velocity(500)
        print('homing right')
        s, d = self.controller.rw_value(1, REFCONF_RAMPMODE, 0)
        while d & 0x10000:
            s, d = self.controller.rw_value(1, REFCONF_RAMPMODE, 0)
        # read latched position
        s, r = self.controller.rw_value(1, X_LATCHED, 0)
        r = sign_extend(r, 24)
        print('found right: ' + str(r))

        # setup left switch for homing
        self.controller.rw_value(0, REFCONF_RAMPMODE, d & ~0x800)
        # init latching mechanism
        self.controller.rw_value(0, X_LATCHED, 0)
        # move in positive direction until limit is hit
        self.controller.set_target_velocity(-500)
        print('homing left')
        s, d = self.controller.rw_value(1, REFCONF_RAMPMODE, 0)
        while d & 0x10000:
            s, d = self.controller.rw_value(1, REFCONF_RAMPMODE, 0)
        # read latched position
        s, l = self.controller.rw_value(1, X_LATCHED, 0)
        l = sign_extend(l, 24)
        print('found left: ' + str(l))
        # back to ramp mode
        self.controller.set_mode(RAMP_MODE)

        # set limits
        self.set_range(a1, a2, l, r)
        # move to 0
        self.set_target_angle(0)
        while not self.is_target_reached():
            pass

    def enable(self):
        '''!@brief      Enables the stepper motor driver
         
        '''
        self.driver.enable()

    def disable(self):
        '''!@brief      Enables the stepper motor driver
         
        '''
        self.driver.disable()

    def set_target_angle(self, deg):
        '''!@brief      Sets the target angle
            @param      deg the degree that the target angle is to be set to
         
        '''
        self.controller.set_target_position(map_range(deg, self.min_angle, self.max_angle,
                                                      self.min_step, self.max_step))

    def get_actual_angle(self):
        '''!@brief      Reads the actual angle
    
        '''
        return map_range(self.controller.get_actual_position(), self.min_step, self.max_step,
                         self.min_angle, self.max_angle)

    # todo: implement and test
    def is_target_reached(self):
        '''!@brief      checks to see if the target position is equal to the actual position
            @return     boolean of whether target has been reached or not. 
         
        '''
        # todo: does this really work (TEST!)
        return self.controller.is_target_reached()
