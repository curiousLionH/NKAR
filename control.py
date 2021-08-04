from serial import Serial, STOPBITS_TWO
from time import time, sleep
from multiprocessing import Process, Queue

STB = 0x76
ID = 0x30

uart = Serial("/dev/ttyTHS1", "115200", stopbits=STOPBITS_TWO)


class Controller:
    mpu_buf = Queue(5)
    read_t = 0

    def __init__(self):
        FLAG = 0b111
        CRC = (ID + FLAG) & 0xff
        uart.write([STB, ID, FLAG, CRC])

        self.read_t = Process(target=self._read_thread)
        self.read_t.daemon = True
        self.read_t.start()

    def move(self, command, speed=None, angle=None):
        if command.upper() == "FORWORD" or command.upper() == "F" or command.upper() == "GO" or command.upper() == "G":
            command = 0x09
        elif command.upper() == "BACKWORD" or command.upper() == "B" or command.upper() == "BACK":
            command = 0x06
        elif command.upper() == "STOP" or command.upper() == "S":
            command = 0x00

        if speed is not None:
            self.speed(speed)

        if angle is not None:
            self.steer(angle)
            sleep(0.2)

        if command == 0x00 or command == 0x06 or command == 0x09:
            CRC = (0x21 + command) & 0xff
            uart.write([STB, 0x21, command, CRC])

    def steer(self, angle):
        if angle < -35:
            angle = -35
        elif angle > 35:
            angle = 35

        correction = (35 / 23)
        angle = (angle) * correction + 2

        angle = int(90 - angle) - 10
        CRC = (0x22 + angle) & 0xff
        uart.write([STB, 0x22, angle, CRC])

        sleep(0.3)

        angle += 10
        CRC = (0x22 + angle) & 0xff
        uart.write([STB, 0x22, angle, CRC])

    def speed(self, speed):
        if speed < 80:
            speed = 80
        elif speed > 255:
            speed = 255

        speed = int(speed)

        CRC = (0x23 + speed) & 0xff
        uart.write([STB, 0x23, speed, CRC])

    def read(self, *devs):
        result = []
        value = self.mpu_buf.get()

        for dev in devs:
            result.append(value[dev])

        return result

    def _read_thread(self):
        data = []
        value = {"ACC_X": 0, "ACC_Y": 0, "ACC_Z": 0,
                 "GYRO_X": 0, "GYRO_Y": 0, "GYRO_Z": 0,
                 "COMP_X": 0, "COMP_Y": 0, "COMP_Z": 0,
                 "ENC": 0, "DIST": 0,
                 "US_F0": 0, "US_F1": 0, "US_F2": 0,
                 "US_R0": 0, "US_R1": 0, "US_R2": 0}

        while True:
            try:
                if uart.readable():
                    uart_list = uart.read()

                    for c in uart_list:
                        data.append(c)

                    if len(data) >= 2:
                        if data[0] == 0x76:
                            if data[1] == 0x31:
                                if len(data) >= 21:  # MPU+AK8975
                                    CRC = int(data[20])
                                    this_CRC = sum(data[1:20]) & 0xff

                                    if CRC == this_CRC:
                                        value["ACC_X"] = int.from_bytes(data[2:4], byteorder='big', signed=True)
                                        value["ACC_Y"] = int.from_bytes(data[4:6], byteorder='big', signed=True)
                                        value["ACC_Z"] = int.from_bytes(data[6:8], byteorder='big', signed=True)
                                        value["GYRO_X"] = int.from_bytes(data[8:10], byteorder='big', signed=True)
                                        value["GYRO_Y"] = int.from_bytes(data[10:12], byteorder='big', signed=True)
                                        value["GYRO_Z"] = int.from_bytes(data[12:14], byteorder='big', signed=True)
                                        value["COMP_X"] = int.from_bytes(data[14:16], byteorder='big', signed=True)
                                        value["COMP_Y"] = int.from_bytes(data[16:18], byteorder='big', signed=True)
                                        value["COMP_Z"] = int.from_bytes(data[18:20], byteorder='big', signed=True)

                                        if self.mpu_buf.full():
                                            try:
                                                self.mpu_buf.get_nowait()
                                            except:
                                                pass

                                        self.mpu_buf.put(value)

                                    for _ in range(21):
                                        data.pop(0)
                            elif data[1] == 0x32:  # Ultrasonic
                                if len(data) >= 9:
                                    CRC = int(data[8])
                                    this_CRC = sum(data[1:8]) & 0xff

                                    if CRC == this_CRC:
                                        value["US_F0"] = int(data[2])
                                        value["US_F1"] = int(data[3])
                                        value["US_F2"] = int(data[4])
                                        value["US_R0"] = int(data[5])
                                        value["US_R1"] = int(data[6])
                                        value["US_R2"] = int(data[7])

                                        if self.mpu_buf.full():
                                            try:
                                                self.mpu_buf.get_nowait()
                                            except:
                                                pass

                                        self.mpu_buf.put(value)

                                    for _ in range(9):
                                        data.pop(0)

                            elif data[1] == 0x33:  # Encoder
                                if len(data) >= 5:
                                    CRC = int(data[4])
                                    this_CRC = sum(data[1:4]) & 0xff

                                    if CRC == this_CRC:
                                        value["ENC"] = int.from_bytes(data[2:4], byteorder='big')
                                        value["DIST"] = value["ENC"] / 1.90986  # 1.59155

                                        if self.mpu_buf.full():
                                            try:
                                                self.mpu_buf.get_nowait()
                                            except:
                                                pass

                                        self.mpu_buf.put(value)

                                    for _ in range(5):
                                        data.pop(0)
                            else:
                                data.pop(1)
                        else:
                            data.pop(0)
            except:
                pass

    def reset_ENC(self):
        CRC = (0x40 + 0xff) & 0xff
        uart.write([STB, 0x40, 0xff, CRC])


'''
r=Controller()

#r.move("G",255,25)

while True:
    print(r.read("GYRO_X", "GYRO_Y", "GYRO_Z","DIST"))
    sleep(0.1)
'''
