"""
Script to record and analyze IMU position update data from DeadReckon class.
"""
import math as m
import time
import numpy as np
from yamspy import MSPy
from dead_reckon import DeadReckon

allPositions = []
accelTicks = []
gyroTicks = []
deadReckon = DeadReckon()
i = 0
numReadings = 1000
serial_port = "/dev/ttyUSB0"
with MSPy(device=serial_port, loglevel='WARNING', baudrate=115200) as board:
    t1 = time.time()
    while i<numReadings:
        if board.send_RAW_msg(MSPy.MSPCodes['MSP_RAW_IMU'], data=[]):
            dataHandler = board.receive_msg()
            board.process_recv_data(dataHandler)
            gyro = board.SENSOR_DATA['gyroscope']
            gyroTicks.append(gyro)
            accel = board.SENSOR_DATA['accelerometer']
            accelTicks.append(accel)
            t2 = time.time()
            dt = t2 - t1                        

            gyro_update = []
            gyro_update = deadReckon.gyro_reckon(gyro,dt)
            t2 = time.time()
            dt = t2 - t1
            
            position, orientation = deadReckon.getPose()
            thisPos = []
            thisPos = deadReckon.accel_reckon(orientation, accel, dt)
            t1 = time.time()
            allPositions.append(list(thisPos))
            i += 1
            print(i)
    accelTicks = np.transpose(accelTicks)
    xTicks = accelTicks[0]
    yTicks = accelTicks[1]
    zTicks = accelTicks[2]     
    #print(accelTicks)

    xTicksMu = np.mean(xTicks)
    print('X Ticks mean ',xTicksMu)
    yTicksMu = np.mean(yTicks)
    print('Y Ticks mean ',yTicksMu)
    zTicksMu = np.mean(zTicks)
    print('Z Ticks mean ',zTicksMu)

    gyroTicks = np.transpose(gyroTicks)
    xTicksGyro = gyroTicks[0]
    yTicksGyro = gyroTicks[1]
    zTicksGyro = gyroTicks[2]     
    #print(accelTicks)

    xTicksMuGyro = np.mean(xTicksGyro)
    print('X Gyro Ticks mean ',xTicksMuGyro)
    yTicksMuGyro = np.mean(yTicksGyro)
    print('Y Gyro Ticks mean ',yTicksMuGyro)
    zTicksMuGyro = np.mean(zTicksGyro)
    print('Z Gyro Ticks mean ',zTicksMuGyro)
    
    x = allPositions[:][0]
    y = allPositions[:][1]
    z = allPositions[:][2]
    
    xMu = np.mean(x)
    xSTD = np.std(x)
    print('X mean ',xMu)
    print('X standard deviation: ',xSTD)
    yMu = np.mean(y)
    ySTD = np.std(y)
    print('Y mean ',yMu)
    print('Y standard deviation: ',ySTD)
    zMu = np.mean(z)
    zSTD = np.std(z)
    print('Z mean ',zMu)
    print('Z standard deviation: ',zSTD)
