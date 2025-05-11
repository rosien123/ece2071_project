import numpy as np
import serial
import serial.tools.list_ports
import time 

"""
This file handles manual sampling from the processing STM (Task 1). The manual_sample function should interact
with the processing STM to generate a numpy array of the specified data. This numpy array is then returned.

Last modified by: Martin
Date last modified: 28/04
Changes made:
"""

def manual_sample(ser, sample_rate, duration):
    prev = None #defining previous as nothing
    for _ in range(duration * sample_rate): #This will read the sample for specified time
        byte = ser.read(1) # Read 1 byte
        if prev is None:
            filtered = byte[0] #Trying to find the average of the first reading won't work so set the previous as the same as the first
        else:
            filtered = (prev + byte[0])/2 #Finding the average between consecutive readings 
        data.append(filtered)# append value into data array
        prev = byte[0] #define previous as previous reading, and not the average value
    
    #Convert into numpy array so it can be converted into audio file later
    data = np.array(data) 
    data = (data - data.min()) / data.max()
    data = data * 255

    data = data.astype(np.uint8)
    return data