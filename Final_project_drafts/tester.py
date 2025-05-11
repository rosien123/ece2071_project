import serial

ser = serial.Serial("COM10", 352800)
while True:
    if ser.in_waiting:
        print(ser.read(1))