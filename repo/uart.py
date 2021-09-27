import os
import sys
import serial
from operator import eq 
import time

autoflag=0
serialPort = serial.Serial ('/dev/ttyTHS1',115200)
sendData=[0x76,0x23,255,0x3E]
speed = 255
serialPort.write(sendData)
time.sleep(1)
sendData[1] = 0x90
sendData[2] = 0x0F
serialPort.write(sendData)
while True:
        autoflag = int(input("1: Auto Move Mode\n2: Manual Move Mode\n3: Program Exit"))
        if autoflag == 1:
                autoflag = 1
                sendData[1] = 0x90
                sendData[2] = 0xF0
                serialPort.write(sendData)
        elif autoflag == 2:
                sendData[1] = 0x90
                sendData[2] = 0x0F
                serialPort.write(sendData)
                while autoflag == 2:
                        select = int(input("1: go\n2: back\n3: left\n4: right\n5: center\n6: stop\n7: speed up\n8: speed down\n9: quit"))
                        if select == 1:
                                sendData[1] = 0x21
                                sendData[2] = 0x09
                        elif select == 2:
                                sendData[1] = 0x21
                                sendData[2] = 0x06
                        elif select == 3:
                                sendData[1] = 0x22
                                sendData[2] = 140
                        elif select == 4:
                                sendData[1] = 0x22
                                sendData[2] = 40
                        elif select == 5:
                                sendData[1] = 0x22
                                sendData[2] = 90
                        elif select == 6:
                                sendData[1] = 0x21
                                sendData[2] = 0x00
                        elif select == 7:
                                sendData[1] = 0x23
                                speed = speed + 30
                                if speed > 100:
                                        speed = 255
                                sendData[2] = speed
                        elif select == 8:
                                sendData[1] = 0x23
                                speed = speed - 30
                                if speed < 0:
                                        speed = 0
                                sendData[2] = speed
                        elif select == 9:
                                sendData[1] = 0x21
                                sendData[2] = 0x00
                                serialPort.write(sendData)
                                autoflag = 0
                                break
                        serialPort.write(sendData)
        elif autoflag == 3:
                sendData[1] = 0x90
                sendData[2] = 0x0F
                serialPort.write(sendData)
                sendData[1] = 0x21
                sendData[2] = 0x00
                serialPort.write(sendData)
                autoflag = 0
                break
sys.exit(0)
