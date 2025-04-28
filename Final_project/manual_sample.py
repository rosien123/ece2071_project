import numpy as np
import serial
import serial.tools.list_ports
import time 

"""
This file handles manual sampling from the processing STM (Task 1). The manual_sample function should interact
with the processing STM to generate a numpy array of the specified data. This numpy array is then returned.

Last modified by: 
Date last modified:
Changes made:
"""

def manual_sample(time, ser, sample_rate):
    for _ in range(time * sample_rate):
        if ser.in_waiting:
            byte = ser.read(1)       # Read 1 byte
            print(byte[0])
            data.append(byte[0])     # append index 0
        
    data = np.array(data)
    data = (data - data.min()) / data.max()
    data = data * 255

    data = data.astype(np.uint8)
    return data