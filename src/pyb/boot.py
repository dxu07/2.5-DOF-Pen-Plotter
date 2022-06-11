# boot.py -- run on boot to configure USB and filesystem
# Put app code in main.py

import machine
import pyb
# import micropython

# ISO 3166-1 Alpha-2 code, eg US, GB, DE, AU
pyb.country('US')

# micropython.alloc_emergency_exception_buf(100)
# pyb.repl_uart(None)

#pyb.main('main.py') # main script to run after this one
#pyb.usb_mode('VCP+MSC') # act as a serial and a storage device
#pyb.usb_mode('VCP+HID') # act as a serial device and a mouse
