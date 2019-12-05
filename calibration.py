import sys, serial
from serial.tools import list_ports
import json
from struct import *
import time
import keyboard
from rgSerial import *

# Variable Definition
current_state = 0			# 0: calibration, 1: teleoperation
cal_motor = 3				# which motor is being calibrated (3, 4, 5)
motor_cpr = 130				# DC motor encoder count per rev
com_baud = 1000000			# communication baud rate
dynamixel_current = 200		# dynamixel current
	
increment = 5
curr_pos = [0, 0, 0, 0, 0, 0, 0, 0, 0]				# motor positions
pos_offset = [135, 135, 135, 0, 0, 0, 0, 0, 0]		# motor position offsets

gripper = []		# list contains all motors

	
def to_next_motor():
	global cal_motor
	cal_motor += 1
	if cal_motor > 5:
		cal_motor -= 3
	print("Calibrate motor %d" % (cal_motor))

def to_teleoperation():
	global current_state
	current_state = 1
	print("switch to teleoperation!")
	
def handle_motor_pos(event):
	global increment
	global curr_pos
	global current_state
	global pos_offset

	# change calibration
	if keyboard.get_hotkey_name() == 'r':
		pos_offset[cal_motor] += 1
		print("motor %d offset = %d" % (cal_motor, pos_offset[cal_motor]))
	if keyboard.get_hotkey_name() == 'l':
		pos_offset[cal_motor] -= 1
		print("motor %d offset = %d" % (cal_motor, pos_offset[cal_motor]))
	for m_id in range(9):
		gripper[m_id].move_to_pos(pos_offset[m_id])
	
keyboard.hook(handle_motor_pos)
keyboard.add_hotkey('enter', to_next_motor)


# Initialize all motors
for ii in range(9):
	# the serail port shouldn't matter here
	new_motor = SBMotor('/dev/tty.usbmodem51011901', ii, motor_cpr, com_baud)
	gripper.append(new_motor)

# Set dynamixel current
for ii_d in range(3):
	gripper[ii_d].set_current(dynamixel_current)


keyboard.wait('.')

	
