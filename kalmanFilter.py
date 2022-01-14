#!/usr/bin/python3
import sys
import pynmea2
import serial
ser = serial.Serial("/dev/ttyS0", 9600, 8, 'N', 1, timeout=1) 
while True:
     print('reading')
     data = ser.readline()
     if sys.version_info[0] == 3:
        data = data.decode("utf-8","ignore")
     if data[0:6] == '$GNGGA':
        print(data)
         