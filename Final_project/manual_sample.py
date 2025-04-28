import numpy as np
import serial
import serial.tools.list_ports
import time 

"""
This file handles manual sampling from the processing STM (Task 1). The manual_sample function should interact
with the processing STM to generate a numpy array of the specified data. This numpy array is then returned.

Last modified by: Martin
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

# import numpy as np
# import wave
# import serial
# import serial.tools.list_ports
# import time


# data = []
# SAMPLE_RATE = 10000

# # List available ports
# devices = serial.tools.list_ports.comports()
# for dev in devices:
#     print(dev)  # Use .device instead of [0] for better compatibility
# # Initialize serial port with proper settings
# ser = serial.Serial("COM11", 115200)

# prev = None
# for _ in range(5 * SAMPLE_RATE): #forming the waveform 
#     byte = ser.read(1)       # Read 1 byte
#     if prev is None:
#         filtered = byte[0]
#     else:
#         filtered = (prev + byte[0])/2
#     # print(byte[0])
#     data.append(filtered)     # append index 0
#     prev = byte[0]
        
# data = np.array(data)
# data = (data - data.min()) / data.max()
# data = data * 255

# data = data.astype(np.uint8)

# file_name = "testing.wav"
# with wave.open(file_name, 'wb') as wav_file:
#     wav_file.setnchannels(1)
#     wav_file.setsampwidth(1)
#     wav_file.setframerate(SAMPLE_RATE)
#     wav_file.writeframes(data.tobytes())