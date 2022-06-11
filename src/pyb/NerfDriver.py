'''!
    @file       NerfDriver.py
    
    @brief      Driver to be used with the pyb board on the nerf gun to control firing. 
    
    @details    This driver interacts directly with the pyb board. It manages the 
                nerf guns integrated firing mechanisms as well as the IR sensor we 
                used to determine if a dart has been fired or not.
                
    @author     Alex Radovan
    @author     Daniel Xu
    @date       6/10/2022
    
'''
import pyb


class OutOfAmmo(Exception):
    '''!@brief      Exception that occurs when gun runs out of ammo                   
    '''
    pass


class BarrelJam(Exception):
    '''!@brief      Exception that occurs when gun jams/ misfires               
    '''
    pass


class Nerf:
    '''!@brief      Driver used to interface with our custom electronic hardware. 
        @details    Objects of this class are used to control the firing mechanisms 
                    that are integrated within the nerf gun. This includes both the 
                    spooling and the trigger mechanisms. 
                    
    '''
    def __init__(self, spool, trig, dart, num_darts=15):
        '''!@brief      Initializes the NERF pyb elements 
            @param      spool is the pin that is associated with the spool up motor
            @param      trig is the pin that is associated with the trigger motor
            @param      dart is the pin that is associated the IR sensor that detects if the dart has been fired
            @param      num_darts defaults to 15, is the number of darts in the gun. 
    
        '''
        self.spool = pyb.Pin(spool, pyb.Pin.OUT_PP, value=0)
        self.trig = pyb.Pin(trig, pyb.Pin.OUT_PP, value=0)

        self.dart = pyb.ExtInt(dart, pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, self.dart_callback)
        self.dart.disable()

        self.dart_flag = 0

        self.num_darts = num_darts

    def dart_callback(self, line):
        '''!@brief      callback if the dark has been fired  
            @param      line IRQ for callback   
            
        '''
        print("fired")
        self.dart_flag = 1

    # todo: minimum and maximum timeout?
    #   yielding - call normally with tuple(fire) or for _ in fire: pass
    def fire(self, stopped):
        '''!@brief      Controls the firing mechanism in the dart.       
            @param      stopped is boolean that determines if firing mechanism needs to stop.     
            
        '''
        # are we out of ammo
        if self.num_darts <= 0:
            raise OutOfAmmo

        # spool up for 5 seconds
        self.spool.value(1)
        m = pyb.millis()
        while pyb.millis() - m < 5000:
            if stopped.get():
                self.spool.value(0)
                return
            yield

        # fire dart
        self.dart_flag = 0
        self.trig.value(1)

        # wait for dart callback or timeout
        self.dart.enable()
        m = pyb.millis()
        while self.dart_flag == 0 and pyb.millis() - m < 3000:
            if stopped.get():
                self.dart.disable()
                self.trig.value(0)
                self.spool.value(0)
                return
            yield
        self.dart.disable()
        # todo: needed?
        pyb.delay(10)
        # stop firing
        self.trig.value(0)
        self.spool.value(0)

        # check for jam
        print(self.dart_flag)
        if self.dart_flag == 0:
            raise BarrelJam

        # decrement ammo count
        self.num_darts -= 1

    def reload(self, num_darts=15):
        '''!@brief      resets the ammo count once the mag has been reloaded     
            @param      num_darts is the number of darts that has been added to magazine. Defaults to 15.  
            
        '''
        self.num_darts = num_darts
