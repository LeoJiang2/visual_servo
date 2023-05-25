#!/usr/bin/env python
"""
Stepper.py
Function: A simple class to operate a Big Easy Driver and control the
extrusion of the soft arm in cm
Author: Ted Loewenthal and Benjamin Walt
Date: 10/18/2020
Purpose: SoftAgBot system integration project
Version: 0.5
"""



"""
Notes about setup:

Assumes stepper connection:
A is Red Blue
B is Green Black

Limit Switch is normally closed (1) when extended, open when retracted

Big Easy Driver is set to FULL step - (M1,M2,M3) all pulled to ground
"""

import RPi.GPIO as GPIO
import time


_SLEEP_TIME = 0.001 # Bigger value, slower the movement
_POSITIVE_DIR = 0
_NEGATIVE_DIR = _POSITIVE_DIR^1 # Must be opposite _POSITIVE_DIR
_RETRACTED = 0 # Limit Switch Pressed
_EXTENDED = 1 # Limit Switch Not Pressed
_RATIO = 250.0 # Number of FULL steps to 1 cm
_STEPPER_OFFSET = 1.0 # Accounts for trimming of the BR2 to fix broken part


class StepperMotor():
	"""Class that controls the stepper motor for extrusion of the soft arm"""
	def __init__(self, step_pin, dir_pin, switch_pin):
		self._step_pin = step_pin
		self._dir_pin = dir_pin
		self._switch_pin = switch_pin
		self._sleep_time = _SLEEP_TIME
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self._step_pin, GPIO.OUT)
		GPIO.setup(self._dir_pin, GPIO.OUT)
		GPIO.setup(self._switch_pin, GPIO.IN)
		self._position = -1 # Holds the position in steps
		self._increment_dir = -1
		self._moving_status = False # False if not moving
		self._rezero()
		self._stepper_test()

	def __del__(self):
		GPIO.cleanup()

	"""
	Check the limit switch value.
	The meaning depends on the limit switch configuration NO or NC and is set in the constants
	_RETRACTED and _EXTENDED
	"""
	def _check_switch(self):
		a = GPIO.input(self._switch_pin)
		return a

	"""
	Drives the stepper in until the limit switvh is activated to return to the zero configuration.
	"""
	def _rezero(self): # Does not change moving status, but this should be fine
		GPIO.output(self._dir_pin, _NEGATIVE_DIR)
		print("Zeroing")
		while self._check_switch() == _EXTENDED:     # lasts until endstop is hit
			GPIO.output(self._step_pin, 1)       # pulse motor
			time.sleep(_SLEEP_TIME)
			GPIO.output(self._step_pin, 0)       # pulse motor
		self._position = 0
		print("Done Zeroing")
	
	"""
	Tests the stepper for proper operation. 
	It extrudes the stepper until the limit switch stops indicating retracted.
	If it makes it to 100 steps and there is no change, it inidcates a failure.
	If not, it passes and the system is rezeroed.
	"""
	def _stepper_test(self):
		self._counter = 0
		GPIO.output(self._dir_pin, _POSITIVE_DIR)
		while self._check_switch() == _RETRACTED:
			GPIO.output(self._step_pin, 1)       # pulse motor
			time.sleep(_SLEEP_TIME)
			GPIO.output(self._step_pin, 0)       # pulse motor
			self._counter += 1
			if self._counter == 100:  # 100 seems to work.  Depends on set up.
				print("Stepper Test Failed")
				return 0
		print("Stepper Test Passed")
		self._rezero()
		
	"""
	Sets the length of the sleep between steps
	Increasing the value slows the extrusion
	Decreasing the value speeds up the extrusion
	There is a limit to how fast it can go.
	"""
	def set_sleep_time(sleep_time_sec):
		sleep_time_sec = max(0.0005, min(0.01, sleep_time_sec)) # Are these good limits?
		self._sleep_time = sleep_time_sec

	"""
	Returns the current length of the extrusion.
	Is based on requested values not actual posiion.
	It is not dynamic and cannot account for skipped steps
	"""
	def get_position_cm(self):
		return self._position/_RATIO       # x steps * 40um per step/10000  yields position in cm
	
	"""
	Returns the status of the stepper.
	True = Currently moving (i.e. it has not reached the requested number of steps)
	False = Not moving
	"""
	def get_status(self):
		return self._moving_status

	"""
	Command to move the stepper to the requested position
	end_position is given in cm
	_STEPPER_OFFSET was created to adjust for a non standard length SCA
	"""
	def move_stepper(self, end_pos):
		self._moving_status = True
		end_pos = max(0.0, min(20.0 - _STEPPER_OFFSET, end_pos))
		end_pos_steps = int(end_pos*_RATIO)
		if end_pos_steps <= 1: 
			self._rezero() # This is to avoid any accumulated error when retracting
		else:
			end_pos_steps += _STEPPER_OFFSET*_RATIO # This should correct for offset due to trimmed BR2
			# It is done here to ensure that a 0 request still full retracts the device
			direction = 0
			if self._position < end_pos_steps:     # positive move
				GPIO.output(self._dir_pin, _POSITIVE_DIR)
				self._increment_dir = 1
				direction = _POSITIVE_DIR
			else:                          # negative move
				GPIO.output(self._dir_pin, _NEGATIVE_DIR)
				self._increment_dir = -1
				direction = _NEGATIVE_DIR

			while end_pos_steps != self._position:
				if self._check_switch()== _RETRACTED and direction == _NEGATIVE_DIR:
					print("error")
					break
				GPIO.output(self._step_pin, 1)   # pulse motor
				time.sleep(_SLEEP_TIME)
				GPIO.output(self._step_pin, 0)   # pulse motor
				self._position += self._increment_dir # update position
		self._moving_status = False
            
