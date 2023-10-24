import time
import serial
import numpy as np
from vpython import *

arduino_data = serial.Serial('com3', 115200)
time.sleep(1)

toRad = np.pi/180.0
toDeg = 1/toRad


# Def your function here
def Q_YawPitchRoll(yaw,pitch,roll):
    Q = np.array([[cos(radians(yaw))*cos(radians(pitch)),
                   cos(radians(yaw))*sin(radians(pitch))*sin(radians(roll))-sin(radians(yaw))*cos(radians(roll)),
                   cos(radians(yaw))*sin(radians(pitch))*cos(radians(roll))+sin(radians(yaw))*sin(radians(roll))],
                  [sin(radians(yaw))*cos(radians(pitch)),
                   sin(radians(yaw))*sin(radians(pitch))*sin(radians(roll))+cos(radians(yaw))*cos(radians(roll)),
                   sin(radians(yaw))*sin(radians(pitch))*cos(radians(roll))-cos(radians(yaw))*sin(radians(roll))],
                  [-sin(radians(pitch)),cos(radians(pitch))*sin(radians(roll)),
                   cos(radians(pitch))*cos(radians(roll))]])
    return Q

def Q_Quaternion(q0,q1,q2,q3):
    Q = np.array([[2*(q0**2 + q1**2)-1,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2)],
                  [2*(q1*q2+q0*q3),2*(q0**2 + q2**2)-1,2*(q2*q3-q0*q1)],
                  [2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),2*(q0**2 + q3**2)-1]])
    
    return Q
counter = 0
while True:
    while arduino_data.inWaiting() == 0:
        pass
    data_packet = arduino_data.readline()
    try:
        data_packet = str(data_packet, 'utf-8')
        counter += 1
        split_packet = data_packet.split(",")
        q0 = float(split_packet[0])
        q1 = float(split_packet[1])
        q2 = float(split_packet[2])
        q3 = float(split_packet[3])
        roll = float(split_packet[4])
        pitch = float(split_packet[5])
        yaw = float(split_packet[6])
        Q1 = Q_YawPitchRoll(yaw, pitch, roll)
        Q2 = Q_Quaternion(q0,q1,q2,q3)
        if counter % 200 == 0:
            print('Q_RollPitchYaw = \n',Q1)
            print('Q_Quaternion = \n',Q2)

    except:
        pass