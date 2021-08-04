import serial
arduino = serial.Serial('/dev/ttyACM0',9600)
#arduino = serial.Serial('/dev/cu.usbmodem14201',115200)
# mega == 14201

# set default value
default = 0
print('Calculating default value', end='')
for i in range(10) :
    data = arduino.readline()
    data = data[:len(data)-2]
    
    default += float(data)

    print('.', end='')
    
print()
default /= 10


# main
while(True):
    data = arduino.readline()
    data = data[:len(data)-2]

    data = float(data)
    diff = int(abs(data-default))

    print('default = ', int(default), 'now = ', int(data), 'diff = ', diff, end = '')

    length = len(str(diff))
    if (length >= 3) :
        print('=> Fog detected!')
        break
    else :
        print()
