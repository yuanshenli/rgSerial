rgSerial
=============================================

### Description

Low level control, and python API for roller gripper V2.
- current based position control for dynamixels, 
- PD contoller for pivot and roller motors
- Tested on macOS Catalina, Ubuntu 16.0.4
- custom serial communication protocol built on top of the protocol developed by [Rohan Agrawal](https://github.com/rohbotics?tab=repositories)

### Hardware
[Dynamixel DYNAMIXEL XM430-W350-T](http://www.robotis.us/dynamixel-xh430-w350-t/)

[Teensy 3.6](http://www.robotis.us/dynamixel-xh430-w350-t/)

[Servocity Micro gear motors](https://www.servocity.com/110-rpm-micro-gear-motor-w-encoder)

### API example
under main in 
```
rgSerial.py
```

### Teleoperation example

```
sudo python teleoperation.py
```
