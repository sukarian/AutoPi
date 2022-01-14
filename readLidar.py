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
            self.write = i2c_msg.write(self.address, [1, 2, 7])
            self.read = i2c_msg.read(self.address, 7)
    
        def readTFMini(self):  
            """ Return the distance value in selected unit. Default: cm """
            

            with SMBus(I2Cbus) as bus:
                bus.i2c_rdwr(self.write, self.read)
                data = list(self.read)

                TrigFlag = data[0]
                Dist = data[3] << 8 | data[2]
                Dist = Dist/100
                Strength = data[5] << 8 | data[4]
                Mode = data[6]

            return [TrigFlag, Dist, Strength, Mode]

if __name__ == "__main__":
    while True:
        alt = Altimeter()
        [trigFlag, dist, strength, mode] = alt.readTFMini()
        print(dist)