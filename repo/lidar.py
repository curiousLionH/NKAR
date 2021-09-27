from serial import Serial, STOPBITS_ONE
from time import time, sleep
from threading import Thread, RLock
from copy import deepcopy
import math

lock=RLock()

quality_limit=10
cycles_size=5
_tmp_cycle=[]
cycles=[]
_tmp_angle={}
angles={}
_tmp_coord=[]
coords=[]
coord_x=[]
coord_y=[]

first_cycle=True

port=Serial('/dev/ttyUSB0',115200,stopbits=STOPBITS_ONE,dsrdtr=True)

class LiDAR:
    read_t=0
    exit_t=False

    def __init__(self, autostart=True):
        if autostart:
            self.start()

    def start(self):
        port.dtr=False
        port.write([0xA5,0x20])
        self.read_t=Thread(target=self._read_thread,daemon=True)
        self.read_t.start()

    def stop(self):
        port.dtr=True
        port.write([0xA5,0x25])
        self.exit_t=True

    def get(self,num=0):
        lock.acquire()
        if len(cycles)>=num+1 :
            value=cycles[num]
        else:
            value=False
        lock.release()
        return value

    def get_dist_by_theta(self):
        lock.acquire()
        value=angles
        lock.release()
        return value

    def get_coords(self):
        value=[]
        while len(value)==0:
            sleep(0.01)
            lock.acquire()
            value=coords
            lock.release()
        return value

    def get_x(self):
        value=[]
        while len(value)==0:
            sleep(0.01)
            lock.acquire()
            value=coord_x
            lock.release()
        return value

    def get_y(self):
        value=[]
        while len(value)==0:
            sleep(0.01)
            lock.acquire()
            value=coord_y
            lock.release()
        return value

    def _read_thread(self):
        buf=[]

        while True:
            if self.exit_t:
                self.exit_t=False
                break

            if port.readable():
                for c in port.read():
                    buf.append(c)

                while len(buf)>=5:
                    s0 = (buf[0] & 0x01 != 0)
                    s1 = (buf[0] & 0x02 != 0)

                    new={"time":0,"start":False,"quality":0,"angle":0,"distance":0,"available":False}

                    if s0 != s1 :
                        if buf[1] & 0x01 == 1 :
                            new["time"]=time()
                            new["start"]=s0
                            new["quality"]=(buf[0] & 0xFF) >> 2
                            new["angle"]=(((buf[1] & 0xFF)  | ((buf[2] & 0xFF) << 8)) >> 1)/64.0
                            new["distance"]=((buf[3] & 0xFF) | ((buf[4] & 0xFF) << 8))/4.0
                            new["available"]=True

                    if new["available"]:
                        lock.acquire()
                        if new["start"]:
                            global first_cycle
                            if not first_cycle:
                                global cycles, angles, coords
                                cycles.insert(0,deepcopy(_tmp_cycle))
                                angles=deepcopy(_tmp_angle)
                                coords=deepcopy(_tmp_coord)
                            else:
                                first_cycle=False
                            _tmp_cycle.clear()
                            _tmp_angle.clear()
                            _tmp_coord.clear()
                            coord_x.clear()
                            coord_y.clear()

                            if len(cycles)>cycles_size :
                                cycles.pop(cycles_size-1)

                        if quality_limit<=new["quality"] and new["distance"]>0:
                            _tmp_cycle.append(new)
                            _tmp_angle[new["angle"]]=new["distance"]
                            x=math.cos(math.radians((new["angle"]+270)%360))*new["distance"]
                            y=-math.sin(math.radians((new["angle"]+270)%360))*new["distance"]
                            coord_x.append(x)
                            coord_y.append(y)
                            _tmp_coord.append({"x":x,"y":y})
                        lock.release()

                        for _ in range(5):
                            buf.pop(0)
                    else:
                        buf.pop(0)