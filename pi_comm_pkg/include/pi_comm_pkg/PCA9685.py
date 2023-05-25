#!/usr/bin/env python
"""
PCA9685.py
Function: A simple class to operate a PCA9685 I2C servo controller on an RPi with Python
PCA9685 is a 16-Channel, 12-bit PWM/Servo Driver
Author: Benjamin Walt
Date: 10/01/2020
Purpose: SoftAgBot system integration project
Version: 0.2
This is a rework of Adafruit and Sparkfun's software:
https://github.com/sparkfun/Qwiic_PCA9685_Py/tree/5ad64fe8a66a436b32a2e539c03e6c8bd35c22ca
https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
https://github.com/adafruit/Adafruit_CircuitPython_ServoKit
https://www.adafruit.com/product/815
"""

import smbus
from time import sleep

_DEVICE_ADDRESS = 0x40

_PCA9685_MODE1 = 0x00
_MODE1_RESTART = 0x80
_SWRST = 0x06
_PCA9685_PRESCALE = 0xFE
_MODE1_AI = 0x20 
_OSCILLATOR_FREQUENCY = 25000000 # Default - Need to calibrate
_SERVO_FREQUENCY = 50 # Standard for most servos
_PCA9685_PRESCALE_MIN = 3  # minimum prescale value
_PCA9685_PRESCALE_MAX = 255 # maximum prescale value

# This is the start point for all the LED registers
# There are 4 registers per LED
# ON Low - 8 bits
# ON High - 4 bits
# OFF Low - 8 bits
# OFF High - 4 bits
_PCA9685_LED0_ON_L = 0x06


class PCA9685:
    """A class used to set up and control the PCA9685 a 16-Channel, 12-bit PWM/Servo Driver"""
    def __init__(self, oscillator_freq = _OSCILLATOR_FREQUENCY):
        self._bus = smbus.SMBus(1) # Channel = 1
        self._address = _DEVICE_ADDRESS
        self._oscillator_freq = oscillator_freq
        # This performs a software reset, note that it is not a normal write
##        self._bus.write_byte(0x00, _SWRST) # Is this needed?
        self._write_reg(_PCA9685_MODE1, _MODE1_RESTART)
        sleep(0.5)
        self._write_reg(_PCA9685_MODE1, _MODE1_AI)    #This sets the auto increment - is it needed?   
        self.set_servo_frequency()
        
    def _write_reg(self, reg, value):
        self._bus.write_byte_data(self._address, reg, value)

    def _read_reg(self, reg):
        value = self._bus.read_byte_data(self._address, reg)
        return value
    
    def _set_sleep_bit(self, value):
        """Sets the sleep bit of the mode 1 register without changing the other bits"""
        sleep_bit = 4 # fifth bit
        current_state = self._read_reg(_PCA9685_MODE1)
        if(value == 1):
            new_state = current_state | (0x01 << sleep_bit)
        else:
            new_state = current_state & ~(0x01 << sleep_bit)
        self._write_reg(_PCA9685_MODE1, new_state)
    
    def set_servo_frequency(self, frequency = _SERVO_FREQUENCY):
        """Sets the frequency that the PWM operates at.  50hz is common for servos.
        It means that the full pulse is 20ms and the duty-cycle is a fraction of that.
        0 is 0ms (0% duty-cycle)
        2048 is 10ms (50% duty-cycle)
        4095 is 20ms (100% duty-cycle)
        """
        prescaleval = round((self._oscillator_freq / (frequency * 4096.0))) - 1
        # Needed???
        if (prescaleval < _PCA9685_PRESCALE_MIN):
            prescaleval = _PCA9685_PRESCALE_MIN
        if (prescaleval > _PCA9685_PRESCALE_MAX):
            prescaleval = _PCA9685_PRESCALE_MAX
        prescale = int(prescaleval)
        self._set_sleep_bit(1) # go to sleep
        self._write_reg(_PCA9685_PRESCALE, prescale) # set the prescaler
        self._set_sleep_bit(0) # Wake up
        sleep(0.5)
        
    def set_pwm(self, channel, on, off):
        """Sets the on and off point for the pwm signal. on is typically just 0 and off is a number less than 4096"""
        self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel, on)
        self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +1, on >> 8)
        self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +2, (off & 0xFF))
        self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +3, off >> 8)
        
    def set_oscillator_freq(self, freq):
        """Sets the oscillator frequency which must be calibrated.  Normally set with the instantiation of the object"""
        self._oscillator_freq = freq
        self.set_servo_frequency()
