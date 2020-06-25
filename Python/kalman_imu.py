# Kalman Filter Implementation for MPU-6050 6DOF IMU
#
# Author: Philip Salmony [pms67@cam.ac.uk]
# Date: 3 August 2018
# 
# Modified by: KHADRAOUI Ibrahim
# Date: 25 Juin 2020
# Description: Get the imu data from an arduino via the serial port

import serial
import numpy as np
from time import sleep, time
import math
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
from itertools import count


#Created a serial communication with the serial port 
myArduino = serial.Serial('COM3', baudrate=115200)

#Functions to get data from the the arduino(serial port) 
def get_acc():
    # ax = self.read_word_2c(0x3b) / 16384.0
    # ay = self.read_word_2c(0x3d) / 16384.0
    # az = self.read_word_2c(0x3f) / 16384.0
    myArduino.write(b'1')
    msg1 = myArduino.readline()
    ax = float(msg1) / 16384.0
    myArduino.write(b'2')
    msg2 = myArduino.readline()
    ay = float(msg2) / 16384.0
    myArduino.write(b'3')
    msg3 = myArduino.readline()
    az = float(msg3) / 16384.0
    return [ax, ay, az]

def get_acc_angles():
    [ax, ay, az] = get_acc()
    phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
    theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
    return [phi, theta]

def get_gyro():
    # gx = read_word_2c(0x43) * math.pi / (180.0 * 131.0)
    # gy = read_word_2c(0x45) * math.pi / (180.0 * 131.0)
    # gz = read_word_2c(0x47) * math.pi / (180.0 * 131.0)
    myArduino.write(b'4')
    msg4 = myArduino.readline()
    gx = float(msg4) * math.pi / (180.0 * 131.0)
    myArduino.write(b'5')
    msg5 = myArduino.readline()
    gy = float(msg5) * math.pi / (180.0 * 131.0)
    myArduino.write(b'6')
    msg6 = myArduino.readline()
    gz = float(msg6) * math.pi / (180.0 * 131.0)
    return [gx, gy, gz]

sleep_time = 0.01

# Initialise matrices and variables
C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
P = np.eye(4)
Q = np.eye(4)
R = np.eye(2)

state_estimate = np.array([[0], [0], [0], [0]])

phi_hat = 0.0
theta_hat = 0.0

# Calculate accelerometer offsets
N = 100
phi_offset = 0.0
theta_offset = 0.0

for i in range(N):
    [phi_acc, theta_acc] = get_acc_angles()
    phi_offset += phi_acc
    theta_offset += theta_acc
    sleep(sleep_time)

phi_offset = float(phi_offset) / float(N)
theta_offset = float(theta_offset) / float(N)

print("Accelerometer offsets: " + str(phi_offset) + "," + str(theta_offset))
sleep(2)

# Measured sampling time
dt = 0.0
start_time = time()

print("Running...")
while True:

    # Sampling time
    dt = time() - start_time
    start_time = time()

    # Get accelerometer measurements and remove offsets
    [phi_acc, theta_acc] = get_acc_angles()
    phi_acc -= phi_offset
    theta_acc -= theta_offset
    
    # Gey gyro measurements and calculate Euler angle derivatives
    [p, q, r] = get_gyro()
    phi_dot = p + math.sin(phi_hat) * math.tan(theta_hat) * q + math.cos(phi_hat) * math.tan(theta_hat) * r
    theta_dot = math.cos(phi_hat) * q - math.sin(phi_hat) * r

    # Kalman filter
    A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
    B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

    gyro_input = np.array([[phi_dot], [theta_dot]])
    state_estimate = A.dot(state_estimate) + B.dot(gyro_input)
    P = A.dot(P.dot(np.transpose(A))) + Q

    measurement = np.array([[phi_acc], [theta_acc]])
    y_tilde = measurement - C.dot(state_estimate)
    S = R + C.dot(P.dot(np.transpose(C)))
    K = P.dot(np.transpose(C).dot(np.linalg.inv(S)))
    state_estimate = state_estimate + K.dot(y_tilde)
    P = (np.eye(4) - K.dot(C)).dot(P)

    phi_hat = state_estimate[0]
    theta_hat = state_estimate[2]

    # Display results
    #print("Phi: " + str(round(phi_hat * 180.0 / math.pi, 1)) + " Theta: " + str(round(theta_hat * 180.0 / math.pi, 1)))
    print("Phi: " + str(phi_hat * 180.0 / math.pi) + " Theta: " + str(theta_hat * 180.0 / math.pi))

    sleep(sleep_time)