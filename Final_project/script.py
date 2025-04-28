import numpy as np
import wave
import serial
import serial.tools.list_ports
import time


data = []
SAMPLE_RATE = 5000

# List available ports
devices = serial.tools.list_ports.comports()
for dev in devices:
    print(dev.device)  # Use .device instead of [0] for better compatibility
# Initialize serial port with proper settings
ser = serial.Serial("COM14", 115200)


for _ in range(5 * SAMPLE_RATE):
    if ser.in_waiting:
        byte = ser.read(1)       # Read 1 byte
        print(byte[0])
        data.append(byte[0])     # append index 0
        
data = np.array(data)
data = (data - data.min()) / data.max()
data = data * 255

data = data.astype(np.uint8)

file_name = "testing.wav"
with wave.open(file_name, 'wb') as wav_file:
    wav_file.setnchannels(1)
    wav_file.setsampwidth(1)
    wav_file.setframerate(SAMPLE_RATE)
    wav_file.writeframes(data.tobytes())