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


def to_calibration():
	global current_state
	current_state = 0
	print("switch to calibration!")
	
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
	# print(keyboard.get_hotkey_name() == "q")
	# change increment
	if keyboard.get_hotkey_name() == '1':
		increment -= 1
		increment = max(0, increment)
		print("current increment %d" % (increment))

	elif keyboard.get_hotkey_name() == '2':
		increment += 1
		increment = min(20, increment)
		print("current increment %d" % (increment))

	# move motors
	if keyboard.get_hotkey_name() == 'x':
		curr_pos[0] += increment
		curr_pos[1] += increment
		curr_pos[2] += increment
	if keyboard.get_hotkey_name() == 'z':
		curr_pos[0] -= increment
		curr_pos[1] -= increment
		curr_pos[2] -= increment
	if keyboard.get_hotkey_name() == 'v':
		curr_pos[3] += increment
		curr_pos[4] += increment
		curr_pos[5] += increment
	if keyboard.get_hotkey_name() == 'c':
		curr_pos[3] -= increment
		curr_pos[4] -= increment
		curr_pos[5] -= increment
	if keyboard.get_hotkey_name() == 'n':
		curr_pos[6] += increment
		curr_pos[7] += increment
		curr_pos[8] += increment
	if keyboard.get_hotkey_name() == 'b':
		curr_pos[6] -= increment
		curr_pos[7] -= increment
		curr_pos[8] -= increment
	if keyboard.get_hotkey_name() == 'm':
		curr_pos[6] += 2*increment
		curr_pos[7] -= increment
		curr_pos[8] -= increment
	if keyboard.get_hotkey_name() == ',':
		curr_pos[6] -= 2*increment
		curr_pos[7] += increment
		curr_pos[8] += increment
	
	if current_state == 1:
		# Send to Serial
		for ii in range(3):
			curr_pos[ii] = min(90, curr_pos[ii])
			curr_pos[ii] = max(0, curr_pos[ii])
			curr_pos[ii+3] = min(90, curr_pos[ii+3])
			curr_pos[ii+3] = max(-90, curr_pos[ii+3])
		for m_id in range(9):
			gripper[m_id].move_to_pos(curr_pos[m_id] + pos_offset[m_id])
			# Receive from Serial
		
		gripper[0].request_vals()
		sensor_val = gripper[0].recv_from_serial()

		if sensor_val != None:
			base0 = sensor_val[0] - pos_offset[0]
			base1 = sensor_val[1] - pos_offset[1]
			base2 = sensor_val[2] - pos_offset[2]
			print([base0, base1, base2])
		else:
			print("sensor_val = None")
		time.sleep(0.05)
	# change calibration
	if keyboard.get_hotkey_name() == 'r':
		pos_offset[cal_motor] += 1
		print("motor %d offset = %d" % (cal_motor, pos_offset[cal_motor]))
	if keyboard.get_hotkey_name() == 'l':
		pos_offset[cal_motor] -= 1
		print("motor %d offset = %d" % (cal_motor, pos_offset[cal_motor]))
	if current_state == 0:
		# set all positions to initial value
		for m_id in range(9):
			gripper[m_id].move_to_pos(pos_offset[m_id])
	
# def exit_routine():
# 	print("exit routine")
# 	gripper[0].ser.close()
# 	sys.exit(0)	
	# quit()

# hotkey definition
# keyboard.hook(handle_offset)
keyboard.hook(handle_motor_pos)
keyboard.add_hotkey('[', to_calibration)
keyboard.add_hotkey(']', to_teleoperation)
keyboard.add_hotkey('enter', to_next_motor)
# keyboard.add_hotkey('z', exit_routine)



# Initialize all motors
for ii in range(9):
	# the serail port shouldn't matter here
	new_motor = SBMotor('/dev/tty.usbmodem51011901', ii, motor_cpr, com_baud)
	gripper.append(new_motor)

# Set dynamixel current
for ii_d in range(3):
	gripper[ii_d].set_current(dynamixel_current)


keyboard.wait('.')

	
