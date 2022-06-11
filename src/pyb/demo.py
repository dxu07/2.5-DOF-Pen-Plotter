from StepperDriver import *
import pyb

from NerfDriver import Nerf, OutOfAmmo, BarrelJam


def demo():
    # SCK2, MISO2, MOSI2
    spi_bus = pyb.SPI(2, pyb.SPI.CONTROLLER, baudrate=1000000, polarity=1, phase=1, firstbit=pyb.SPI.MSB)
    print(spi_bus)

    # TIM1_CH2N -> PB0
    tmr = pyb.Timer(1, period=3, prescaler=0)
    clk = pyb.Pin('B0', pyb.Pin.OUT_PP)
    # 20 MHz clock
    ch = tmr.channel(2, pin=clk, mode=pyb.Timer.PWM, pulse_width=2)

    # Stepper driver instance for both DOF
    polar = StepperDriver(spi_bus, 'C6', 'C7')
    azimuthal = StepperDriver(spi_bus, 'B6', 'B7')

    # read the type version from each controller
    s1, d1 = polar.controller.rw_value(1, TYPE_VERSION, 0)
    s2, d2 = azimuthal.controller.rw_value(1, TYPE_VERSION, 0)

    # expected value = 0x429101
    print('Polar Status: ' + '0x{:02x}'.format(s1) + ' Polar Data: ' + '0x{:02x}'.format(d1))
    print('Azimuthal Status: ' + '0x{:02x}'.format(s2) + ' Azimuthal Data: ' + '0x{:02x}'.format(d2))

    # always start at position zero
    #   todo: replace with homing/limit switches
    #polar.controller.rw_value(0, X_ACTUAL, 0)
    #azimuthal.controller.rw_value(0, X_ACTUAL, 0)

    # enable the TMC2208
    polar.enable()
    azimuthal.enable()

    nerf = Nerf('C2', 'C3', 'C0', 5)

    input()

    polar.calibrate(-83, 83)
    # -24,24
    azimuthal.calibrate(-28, 20)

    # demo movement
    while 1:
        i = input('Enter desired polar angle, azimuthal angle, and if to fire (theta,phi,fire): ')
        angles = i.strip().split(',')
        if len(angles) != 3:
            continue
        try:
            theta = float(angles[0])
            phi = int(angles[1])
            fire = int(angles[2])
        except ValueError:
            break

        # move to target angles
        polar.set_target_angle(theta)
        azimuthal.set_target_angle(phi)

        # wait for target to be reached before prompting for new input
        while not polar.is_target_reached() and not azimuthal.is_target_reached():
            pass

        # fire if set
        if fire:
            try:
                nerf.fire()
            except OutOfAmmo:
                input("IM RELOADING!!!!!!!!")
                nerf.reload(5)
            except BarrelJam:
                print("HELP STEPBRO IM STUCK...")
