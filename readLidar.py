from smbus2 import SMBus, i2c_msg
from dead_reckon import DeadReckon
import math as m
import time
import numpy as np
from yamspy import MSPy

class Altimeter:
        def __init__(self):
            self.address = 0x10
            self.I2Cbus = 1
            self.Dist = 0
            self.write = i2c_msg.write(self.address, [1, 2, 7])
            self.read = i2c_msg.read(self.address, 7)
            self.deadReckon = DeadReckon(0,0,0)
    
        def readTFMini(self):  
            with SMBus(self.I2Cbus) as bus:
                bus.i2c_rdwr(self.write, self.read)
                data = list(self.read)
                #TrigFlag = data[0]
                self.Dist = data[3] << 8 | data[2]
                self.Dist = self.Dist/100
                #Strength = data[5] << 8 | data[4]
                #Mode = data[6]

            #return [TrigFlag, Dist, Strength, Mode]

        def readTrueAlt(self, gyro, accel):
            self.readTFMini()
            self.deadReckon.updatePose(gyro, accel)
            pos, [roll, pitch, yaw] = self.deadReckon.getPose()
            roll = m.radians(roll)
            pitch = m.radians(pitch)
            self.Dist = self.Dist * m.cos(roll) * m.cos(pitch)
            #return self.Dist

if __name__ == "__main__":
    alt = Altimeter()
    serial_port = "/dev/ttyUSB0"
    with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200) as board:                
            while True:
                if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
                    gyro = board.SENSOR_DATA['gyroscope']
                    accel = board.SENSOR_DATA['accelerometer']
                    alt.readTrueAlt(gyro, accel)
                    print(alt.Dist)

