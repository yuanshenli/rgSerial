import sys, serial
from serial.tools import list_ports
import json
from struct import *
import time


class MotorSerial:
	ser = None
	VENDOR_ID = "16C0"
	PRODUCT_ID = "0483"
	SERIAL_NUMBER = "5101190"
	target_string = "USB VID:PID=%s:%s SER=%s"%(VENDOR_ID, PRODUCT_ID, SERIAL_NUMBER)

	def __init__(self, serial_port, baud_rate):
		if MotorSerial.ser is None:
			# teensyPort = self.getTeensyPort()
			teensyPort = None
			print(list_ports.comports())
			for port in list(list_ports.comports()):
				if MotorSerial.target_string in port[2]:
					teensyPort = port[0]
			if teensyPort is None:
				print("Automatic serial port connection failed, using custom serial port...")
				MotorSerial.ser = serial.Serial(serial_port, baud_rate)
			else:
				print("port found:", teensyPort)
				MotorSerial.ser = serial.Serial(teensyPort, baud_rate)
			# MotorSerial.ser.open()


class SBMotor:

	def __init__(self, serial_port, motor_id, motor_rpm, baud_rate = 115200):
		self.serial_port = serial_port
		self.motor_id = motor_id
		self.motor_rpm = motor_rpm
		self.baud_rate = baud_rate

		self.motor_reg_space = 16

		self.counter = 0
		self.time = time.time()

		MotorSerial(self.serial_port, self.baud_rate)
		self.ser = MotorSerial.ser

		self.recv_buffer = bytearray(58)
		self.enable = False
		self.stop = False
		self.pos = 0.0
		self.curr = 0.0
		(self.ticks_per_rev, self.kp, self.ki, self.kd) = self.get_default_params()
		print(self.ticks_per_rev, self.kp, self.ki, self.kd)
		self.init(self.ticks_per_rev, self.kp, self.ki, self.kd)


	def get_default_params(self):
		if self.motor_rpm == 26:
			params = (5462.22, 25.0, 0, 0.16)
		elif self.motor_rpm == 44:
			params = (3244.188, 15.0, 0, 0.2)
		elif self.motor_rpm == 52:
			params = (2774.64, 16, 0, 0.3)
		elif self.motor_rpm == 280:
			params = (514.14, 25.0, 0, 0.16)
		elif self.motor_rpm == 2737:
			params = (52.62, 100.0, 0, 0.16)
		elif self.motor_rpm == 130:
			params = (3040.7596, 14.0, 0, 0.3)
		else:
			params = ()
			print("motor not supported")
		return params

	def send_cmd_four_double(self, register_name, val1, val2, val3, val4):
		# ser = serial.Serial(self.serial_port, self.baud_rate)
		data = bytearray(pack('<BBBBHddddxx', 0x7E, 0, 0xFF, 0xAA, register_name, val1, val2, val3, val4))
		data = self.msg_stuff(data)
		self.ser.write(data)

	def send_cmd_single_double(self, register_name, val):
		# ser = serial.Serial(self.serial_port, self.baud_rate)
		data = bytearray(pack('<BBBBHdxx', 0x7E, 0, 0xFF, 0xAA, register_name, val))
		data = self.msg_stuff(data)
		self.ser.write(data)

	def send_cmd_single_int(self, register_name, val):
		# ser = serial.Serial(self.serial_port, self.baud_rate)
		data = bytearray(pack('<BBBBHixx', 0x7E, 0, 0xFF, 0xAA, register_name, val))
		data = self.msg_stuff(data)
		self.ser.write(data)

	def send_cmd_single_bool(self, register_name, val):
		ser = serial.Serial(self.serial_port, self.baud_rate)
		data = bytearray(pack('<BBBBH?xx', 0x7E, 0, 0xFF, 0xAA, register_name, val))
		data = self.msg_stuff(data)
		self.ser.write(data)

	def init(self, ticks_per_rev, kp, ki, kd):
		self.ticks_per_rev = ticks_per_rev
		self.kp = kp
		self.ki = ki
		self.kd = kd
		register_name = self.motor_id * self.motor_reg_space + 16 + 9
		self.send_cmd_four_double(register_name, self.ticks_per_rev, self.kp, self.ki, self.kd)

	def set_enable(self, if_enable):
		register_name = self.motor_id * self.motor_reg_space + 16
		self.enable = if_enable
		self.send_cmd_single_bool(register_name, self.enable)

	def set_stop(self, if_stop):
		register_name = self.motor_id * self.motor_reg_space + 16 + 7
		self.stop = if_stop
		self.send_cmd_single_bool(register_name, self.stop)

	def move_to_pos(self, degree):
		register_name = self.motor_id * self.motor_reg_space + 16 + 1
		self.pos = degree
		self.send_cmd_single_double(register_name, self.pos)

	def set_current(self, curr):
		register_name = self.motor_id * self.motor_reg_space + 16 + 10
		self.curr = curr
		self.send_cmd_single_int(register_name, self.curr)

	def move(self, degree):
		register_name = self.motor_id * self.motor_reg_space + 16 + 1
		self.pos = self.pos + degree
		self.send_cmd_single_double(register_name, self.pos)

	def set_kp(self, kp):
		register_name = self.motor_id * self.motor_reg_space + 16 + 3
		self.kp = kp
		self.send_cmd_single_double(register_name, self.kp)

	def set_ki(self, ki):
		register_name = self.motor_id * self.motor_reg_space + 16 + 4
		self.ki = ki
		self.send_cmd_single_double(register_name, self.ki)

	def set_kd(self, kd):
		register_name = self.motor_id * self.motor_reg_space + 16 + 5
		self.kd = kd
		self.send_cmd_single_double(register_name, self.kd)

	def request_vals(self):
		register_name = 0xa0
		self.send_cmd_single_int(register_name, 53)

	@staticmethod
	def msg_stuff(msg):
		msg_len = len(msg)
		msg[1] = msg_len - 2
		stuffing = 2
		for ii in range(1, msg_len):
			# print("%x"%msg[ii])
			if msg[ii] == 0x7E:
				# print(ii)
				msg[stuffing] = ii
				stuffing = ii
		msg[stuffing] = 0xFF
		return msg

	@staticmethod
	def msg_unstuff(msg):
		stuffing = 2
		while msg[stuffing] != 0xFF:
			tmp = msg[stuffing]

			msg[stuffing] = 0x7E
			stuffing = tmp
			# print(len(msg))
			# print(stuffing)
		msg[stuffing] = 0x7E
		return msg

	def recv_from_serial(self):
		last_state = -1
		cur_state = 0
		rx_len = 0
		idx = 0
		# ready_to_return = False
		return_val = None

		while self.ser.in_waiting:
			# rx = self.ser.read(1)

			if cur_state == 0:
				# if last_state != cur_state:
				# 	print("read header")
				rx = self.ser.read(1)
				# print("%x" % rx[0])
				if int(rx[0]) == 0x7E:
					cur_state = 1
					idx = 0
					self.recv_buffer[idx] = rx[0]
				last_state = 0


			elif cur_state == 1:
				# if last_state != cur_state:
				# 	print("read length")
				rx = self.ser.read(1)
				rx_len = int(rx[0])
				# print("%x" % rx[0])
				if rx_len <= 0:
					cur_state = 3
				else:
					self.recv_buffer[1] = rx[0]
					cur_state = 2
				last_state = 1

			elif cur_state == 2:
				# if last_state != cur_state:
				# 	print("read data")
				rx = self.ser.read(1)
				# print("%x" % rx[0])
				if int(rx[0]) == 0x7E:
					cur_state = 3
				else:
					self.recv_buffer[idx + 2] = rx[0]
					idx += 1
					# print(idx)
					if idx + 2 == rx_len:
						self.recv_buffer = self.msg_unstuff(bytearray(self.recv_buffer))

						unpacked_data = unpack('<BBBBHlffffffffffff', self.recv_buffer)
						return_val = unpacked_data[6:]
						# print(unpacked_data)
						
						# cur_time = time.time()
						# if (cur_time - self.time) > 0.05:
						# 	print(unpacked_data)
						# 	# motor0.counter = 0
						# 	# print(cur_time - self.time)
						# 	self.time = cur_time
						cur_state = 0
						self.counter += 1

				last_state = 2
			if cur_state == 3:
				# if last_state != cur_state:
				# 	print("read error")
				last_state = cur_state
				cur_state = 0
		return return_val


if __name__ == "__main__":

	motor_cpr = 130
	com_baud = 1000000
	dynamixel_current = 200
	curr_pos = 0
	increment = 5
	minDynamixel = 135
	pos_offset = [135, 135, 135, 0, 0, 0, 0, 0, 0]

	gripper = []
	for ii in range(9):
		new_motor = SBMotor('/dev/tty.usbmodem51011901', ii, motor_cpr, com_baud)
		gripper.append(new_motor)

	for ii_d in range(3):
		gripper[ii_d].set_current(dynamixel_current)

	while True:
		# Send to Serial
		for m_id in range(9):
			gripper[m_id].move_to_pos(curr_pos + pos_offset[m_id])
		print(curr_pos)
		# Receive from Serial
		gripper[0].request_vals()
		sensor_val = gripper[0].recv_from_serial()
		# print()

		# Update pos
		time.sleep(0.05)
		curr_pos = curr_pos + increment
		if curr_pos >= 45 or curr_pos <= 0:
			increment = -increment

	# motor0 = SBMotor('/dev/tty.usbmodem51011901', 0, motor_cpr, com_baud)
	# motor1 = SBMotor('/dev/tty.usbmodem51011901', 1, motor_cpr, com_baud)
	# motor2 = SBMotor('/dev/tty.usbmodem51011901', 2, motor_cpr, com_baud)
	# motor3 = SBMotor('/dev/tty.usbmodem51011901', 3, motor_cpr, com_baud)
	# motor4 = SBMotor('/dev/tty.usbmodem51011901', 4, motor_cpr, com_baud)
	# motor5 = SBMotor('/dev/tty.usbmodem51011901', 5, motor_cpr, com_baud)
	# motor6 = SBMotor('/dev/tty.usbmodem51011901', 6, motor_cpr, com_baud)
	# motor7 = SBMotor('/dev/tty.usbmodem51011901', 7, motor_cpr, com_baud)
	# motor8 = SBMotor('/dev/tty.usbmodem51011901', 8, motor_cpr, com_baud)
	# motor0.set_current(100)
	# motor1.set_current(100)
	# motor2.set_current(100)


	# while True:
	# 	motor0.move_to_pos(curr_pos + minDynamixel)
	# 	motor1.move_to_pos(curr_pos + minDynamixel)
	# 	motor2.move_to_pos(curr_pos + minDynamixel)
	# 	motor3.move_to_pos(curr_pos)
	# 	motor4.move_to_pos(curr_pos)
	# 	motor5.move_to_pos(curr_pos)
	# 	motor6.move_to_pos(curr_pos)
	# 	motor7.move_to_pos(curr_pos)
	# 	motor8.move_to_pos(curr_pos)
	# 	# print(curr_pos)
	# 	motor0.request_vals()
	# 	motor0.recv_from_serial()
	# 	time.sleep(1)
	# 	curr_pos = curr_pos + increment
	# 	if curr_pos >= 45 or curr_pos <= 0:
	# 		increment = -increment
		

