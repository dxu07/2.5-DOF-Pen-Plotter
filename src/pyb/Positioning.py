'''!
    @file       Positioning.py
    
    @brief      This program handle the stepper motor driving and the firing of the nerf gun. 
    
    @details    This program interacts directly with Positioning.py and NerfDriver.py to move and shoot the nerf gun. 
                It takes in data from ProcessHPGL or commands from UserInput to determine where to move the gun and if 
                it should shoot. 
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''
import pyb
from StepperDriver import StepperDriver, TYPE_VERSION
from NerfDriver import Nerf, OutOfAmmo, BarrelJam


def is_point(queues):
    '''!@brief      This function checks if there is anything in the queue
        @param      queues is the queue of data
        @return     returns a boolean of whether or not the queues are all empty. 
    '''
    p, a, f = queues
    return p.any() and a.any() and f.any()


def unpack_point(queues):
    '''!@brief      This function unpacks the points from the queues
        @param      queues is the queue of data
        @return     pp the polar position
        @return     aa the azimuthal position
        @return     ff the firing instructions
    '''
    # unpack tuple
    p, a, f = queues
    # get values from queues
    pp, aa, ff = p.get(), a.get(), f.get()
    # dummy value to block
    p.put(0.0)
    # return values
    return pp, aa, ff


def task_positioning(queues, paused, stopped):
    '''!@brief      This function is a task that controls the motors and the nerf gun. 
        @details    This function handles the SPI controlling of the stepper motors. 
                    It interfaces with the Stepper Driver to read and write commands to it. 
                    This allows it to control each motor independently. Additionally, it 
                    handles the firing of the nerf gun.                  
        @param      queues is the queue of shared data for positioning
        @param     paused is the boolean determining whether the paused command has been sent
        @param      stopped is the boolean determining whether the stopped command has been sent
    '''
    # SCK2, MISO2, MOSI2
    spi_bus = pyb.SPI(2, pyb.SPI.CONTROLLER, baudrate=1000000, polarity=1, phase=1, firstbit=pyb.SPI.MSB)

    # Stepper driver instance for both DOF
    polar = StepperDriver(spi_bus, 'C6', 'C7')
    azimuthal = StepperDriver(spi_bus, 'B6', 'B7')

    # TIM1_CH2N -> PB0
    tmr = pyb.Timer(1, period=3, prescaler=0)
    clk = pyb.Pin('B0', pyb.Pin.OUT_PP)
    # 20 MHz clock
    ch = tmr.channel(2, pin=clk, mode=pyb.Timer.PWM, pulse_width=2)

    # read the type version from each controller
    s1, d1 = polar.controller.rw_value(1, TYPE_VERSION, 0)
    s2, d2 = azimuthal.controller.rw_value(1, TYPE_VERSION, 0)

    # expected value = 0x429101
    print('Polar Status: ' + '0x{:02x}'.format(s1) + ' Polar Data: ' + '0x{:02x}'.format(d1))
    print('Azimuthal Status: ' + '0x{:02x}'.format(s2) + ' Azimuthal Data: ' + '0x{:02x}'.format(d2))

    # enable drivers
    polar.enable()
    azimuthal.enable()
    stopped.put(0)

    # instantiate nerf
    nerf = Nerf('C2', 'C3', 'C0', 15)

    # calibrate initially
    #   todo: calibrate on all draws?
    polar.calibrate(-83, 83)
    azimuthal.calibrate(-28, 20)

    # yield after setup
    yield

    # main task loop
    while 1:
        # if stopped
        if stopped.get():
            polar.disable()
            azimuthal.disable()
        # if there is a point to move to
        elif is_point(queues):
            # get point from queue
            p, a, f = unpack_point(queues)
            # enable drivers
            polar.enable()
            azimuthal.enable()
            # start move
            polar.set_target_angle(p)
            azimuthal.set_target_angle(a)
            # wait for move to complete, updating point if needed
            while not polar.is_target_reached() and not azimuthal.is_target_reached():
                # e-stop
                if stopped.get():
                    polar.disable()
                    azimuthal.disable()
                    # ensure nerf doesn't fire
                    f = 0
                    break
                # check if all three queues have a point
                if is_point(queues):
                    p, a, f = unpack_point(queues)
                    polar.set_target_angle(p)
                    azimuthal.set_target_angle(a)
                yield
            # fire if set
            if f:
                try:
                    yield from nerf.fire(stopped)
                except OutOfAmmo:
                    # todo: notify
                    #   once reloaded, re-calibrate
                    paused.put(1)
                    yield
                    polar.calibrate(-83, 83)
                    azimuthal.calibrate(-28, 20)
                    nerf.reload(15)
                except BarrelJam:
                    paused.put(1)
                    yield
                    polar.calibrate(-83, 83)
                    azimuthal.calibrate(-28, 20)
                    nerf.reload(10)
                    # todo: notify
                    pass
            # unblock
            #   todo: only unblock when no exception
            queues[0].clear()
        yield
