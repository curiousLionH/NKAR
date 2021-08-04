import serial
import time
from threading import Thread, RLock
import re

lock=RLock()
arduino = serial.Serial('/dev/ttyACM0', 9600)
fog_exist = False
data = 0
default = 0
diff = 0

class sensor:
    read_t = 0
    exit_t = False

    # set default value
    def __init__(self, autostart=True):
        if autostart:
            self.start()

    def start(self):
        self.read_t=Thread(target=self._read_thread,daemon=True)
        self.read_t.start()

    def stop(self):
        self.exit_t=True

    def get(self):
        lock.acquire()
        value = fog_exist
        lock.release()
        return value

    
    def _read_thread(self):
        global data, diff, default, fog_exist
        i = 0
        while i < 10:
            if arduino.readable():
                data = arduino.readline()
                data = data[:len(data) - 2]
                try : 
                    
                    data = float(data)
                    default =  default + float(data)
                    i = i + 1
                    print(".")
                except ValueError :
                    print("wow")
            else:
                print("arduino is not readable")
        print()
        default /= 10
        
        while True:
            if arduino.readable():
                data = arduino.readline()
                data = data[:len(data) - 2]
                try :
                    data = float(data)
                    diff=int(abs(data - default))
                    print('default = ', int(default), 'now = ', int(data), 'diff = ', diff)

                except ValueError :
                    print("amazing")

                if diff < 100:
                    fog_exist = False
                else:
                    fog_exist = True



