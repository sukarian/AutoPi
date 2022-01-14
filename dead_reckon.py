import math as m
import time
import numpy as np
from yamspy import MSPy
import matplotlib.pyplot as plt

class DeadReckon:
    
    def __init__(self, initLat, initLong, initCourse):
        self.position = [initLat,initLong,0]
        self.orientation = [0,0,initCourse]
        #self.noise = [2.11, 7.789, 513]
        self.noise = [-6, 8, 514]
        self.gyro_noise = [-5, 2, -2]

    def gyro_reckon(self, gyro, dt):
        gyro_constant = 0.075
        gyro_update = gyro_constant * np.array(gyro) * dt
        self.orientation -= gyro_update
        return gyro_update
        
    def body2world(self, orient):
        orient = np.subtract(orient,self.gyro_noise)
        yr = m.radians(orient[0])
        xr = m.radians(orient[1])
        zr = m.radians(orient[2])

        rotz = [[m.cos(zr), m.sin(zr), 0], [-m.sin(zr), m.cos(zr), 0], [0, 0, 1]]
        roty = [[m.cos(yr), 0, -m.sin(yr)], [0, 1, 0], [m.sin(yr), 0, m.cos(yr)]]
        rotx = [[1, 0, 0], [0, m.cos(xr), m.sin(xr)], [0, -m.sin(xr), m.cos(xr)]]
        R = np.cross(np.cross(rotz,roty),rotx)
        return R
        
    def accel_reckon(self, orient, accel, dt):
        accel = np.subtract(accel,self.noise)
        pos = 0.5 * accel * dt ** 2 
        R = self.body2world(orient)
        position_update = np.matmul(R, np.transpose(pos))
        self.position -= position_update
        #self.position *= -1
        return position_update

    def getPose(self):
        return self.position, self.orientation

    def dead_reckon_loop(self):
        serial_port = "/dev/ttyUSB0"
        i = 1
        allPos = []
        try:
           with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200) as self.board:
                t1 = time.time()
                while True:
                    if self.board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
                        dataHandler = self.board.receive_msg()
                        self.board.process_recv_data(dataHandler)
                        gyro = self.board.SENSOR_DATA['gyroscope']
                        accel = self.board.SENSOR_DATA['accelerometer']
                        t2 = time.time()
                        dt = t2 - t1                        
                        
                        gyro_update = []
                        gyro_update = self.gyro_reckon(gyro,dt)
                        t2 = time.time()
                        dt = t2 - t1
                        
                        pos_update = []
                        pos_update = self.accel_reckon(self.orientation, accel, dt)
                        allPos.append(self.position)
                        t1 = time.time()
                        print(self.position)
                        
        except KeyboardInterrupt:
            plt.figure()           
            plt.plot(self.position[:][0], self.position[:][1])
            plt.show()
            pass
if __name__ == "__main__":
    dead_reckoner = DeadReckon(0,0,0)
    dead_reckoner.dead_reckon_loop()

