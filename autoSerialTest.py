# References
# https://stackoverflow.com/questions/24214643/python-to-automatically-select-serial-ports-for-arduino
# https://forum.pjrc.com/threads/25295-Automatically-find-a-Teensy-Board-with-Python-and-PySerial

# Shenli: Minor changes, works on Teensy 3.6 + Mac Os


# from serial.tools import list_ports
# ports = list(list_ports.comports())

# for p in ports:
# 	print(p[2])

import sys, serial
from serial.tools import list_ports

# Teensy USB serial microcontroller program id data:
VENDOR_ID = "16C0"
PRODUCT_ID = "0483"
SERIAL_NUMBER = "5101190"

target_string = "USB VID:PID=%s:%s SER=%s"%(VENDOR_ID, PRODUCT_ID, SERIAL_NUMBER)

def getTeensyPort():
	for port in list(list_ports.comports()):
		print(port[2])
		print(target_string)
		if target_string in port[2]:
			return port[0]

if __name__ == "__main__":
	TeensyPort = getTeensyPort()
	if TeensyPort:
		print("Teensy found on port %s"%TeensyPort)
	else:
		print("No compatible Teensy found. Aborting.")
		sys.exit(1)