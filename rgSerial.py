import serial
import json
from struct import *
import time


class MotorSerial:
	ser = None

	def __init__(self, serial_port, baud_rate):
		if MotorSerial.ser is None:
			MotorSerial.ser = serial.Serial(serial_port, baud_rate)
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

		self.recv_buffer = bytearray(44)
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
			params = (3040.7596, 16.0, 0, 0.3)
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
					if idx == rx_len:
						self.recv_buffer = self.msg_unstuff(bytearray(self.recv_buffer))

						unpacked_data = unpack('<BBBBHlfffffffffxx', self.recv_buffer)
						if motor0.counter == 100:
							print(unpacked_data)
							motor0.counter = 0

							cur_time = time.time()
							print(cur_time - self.time)
							self.time = cur_time

						cur_state = 0
						self.counter += 1


				last_state = 2
			if cur_state == 3:
				# if last_state != cur_state:
				# 	print("read error")
				last_state = cur_state
				cur_state = 0

if __name__ == "__main__":

	motor0 = SBMotor('/dev/tty.usbmodem51011901', 0, 130, 115200)
	motor1 = SBMotor('/dev/tty.usbmodem51011901', 1, 130, 115200)
	motor2 = SBMotor('/dev/tty.usbmodem51011901', 2, 130, 115200)
	motor3 = SBMotor('/dev/tty.usbmodem51011901', 3, 130, 115200)

	# motor0 = SBMotor('/dev/tty.usbmodem4737660', 0, 52, 115200)
	# motor1 = SBMotor('/dev/tty.usbmodem4737660', 1, 280, 115200)
	motor0.move(36000)
	motor1.move(36000)
	motor2.move(36000)
	motor3.move(36000)
	start_time = time.time()
	while True:
		motor0.recv_from_serial()
# print(motor0.counter)

# if motor0.counter > 200:
# 	cur_time = time.time()
# 	print(cur_time - start_time)
# 	start_time = cur_time
# 	motor0.counter = 0


"""
motor0.set_stop(True)
motor1.set_stop(True)
motor2.set_stop(True)
motor3.set_stop(True)

motor0.set_stop(False)
motor1.set_stop(False)
motor2.set_stop(False)
motor3.set_stop(False)

motor0.move(3600)
motor1.move(3600)
motor2.move(3600)
motor3.move(3600)

motor0.move(-360)
motor1.move(-360)
motor2.move(-360)
motor3.move(-360)

motor0.move_to_pos(36000)
motor1.move_to_pos(36000)
motor2.move_to_pos(36000)
motor3.move_to_pos(36000)

motor0.move_to_pos(0)
motor1.move_to_pos(0)
motor2.move_to_pos(0)
motor3.move_to_pos(0)
"""





	# while True:
	# 	data = ser.read(1)  # the last bit gets rid of the new-line chars
	# 	print("%x"%data[0])
	# 	if data == b'\n':
	# 		break
	#
	# ser.close()



	
	
	
	
