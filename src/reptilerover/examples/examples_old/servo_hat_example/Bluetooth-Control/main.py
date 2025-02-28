#!/usr/bin/python
# -*- coding:utf-8 -*-
import os
import serial
import time
from PCA9685 import PCA9685
import subprocess
import threading
 
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

Pos0 = 1500  
Pos1 = 1500 
Pos2 = 1500 
Pos3 = 1500 
Step0 = 0  
Step1 = 0  
Step2 = 0  
Step3 = 0  
pwm.setServoPulse(0,Pos0)
pwm.setServoPulse(1,Pos1)
pwm.setServoPulse(2,Pos2)
pwm.setServoPulse(3,Pos3)

#os.system("echo \"discoverable on\" | bluetoothctl")
BT = serial.Serial("/dev/rfcomm0",115200)
print('serial test start ...')
BT.flushInput()

def timerfunc():
    global Step0,Step1,Step2,Step3,Pos0,Pos1,Pos2,Pos3,pwm

    if(Step0 != 0):
        Pos0 += Step0
        if(Pos0 >= 2500): 
            Pos0 = 2500
        if(Pos0 <= 500):
            Pos0 = 500
        #set channel 0
        pwm.setServoPulse(0,Pos0)    
        
    if(Step1 != 0):
        Pos1 += Step1
        if(Pos1 >= 2500): 
            Pos1 = 2500
        if(Pos1 <= 500):
            Pos1 = 500
        #set channel 1
        pwm.setServoPulse(1,Pos1)   

    if(Step2 != 0):
        Pos2 += Step2
        if(Pos2 >= 2500): 
            Pos2 = 2500
        if(Pos2 <= 500):
            Pos2 = 500
        #set channel 2
        pwm.setServoPulse(2,Pos2)   

    if(Step3 != 0):
        Pos3 += Step3
        if(Pos3 >= 2500): 
            Pos3 = 2500
        if(Pos3 <= 500):
            Pos3 = 500
        #set channel 3
        pwm.setServoPulse(3,Pos3)   

    global t        #Notice: use global variable!
    t = threading.Timer(0.02, timerfunc)
    t.start()

t = threading.Timer(0.02, timerfunc)
t.setDaemon(True)
t.start()

try:
    while True:
        data = ""
        while BT.inWaiting() > 0:
            data += BT.read(BT.inWaiting()).decode('utf-8')
        if data != "":
            print(data)
            if not data:   
                break  
            if data == "Stop":
                Step0 = 0
                Step1 = 0
                Step2 = 0
                Step3 = 0
                print("1")
            elif data == "Forward":
                Step0 = -5
                print("2")
            elif data == "Backward":
                Step0 = 5
                print("3")
            elif data == "TurnLeft":
                Step1 = -5
                print("4")
            elif data == "TurnRight":
                Step1 = 5
                print("5")
            elif data == "Up":
                Step2 = -5
                print("6")
            elif data == "Down":
                Step2 = 5
                print("7")
            elif data == "Left":
                Step3 = 5
                print("8")
            elif data == "Right":
                Step3 = -5
                print("9")
        time.sleep(0.0001)
except KeyboardInterrupt:
    if BT!= None:
        BT.close()
