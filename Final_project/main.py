import numpy as np
import wave
import serial
import serial.tools.list_ports
import time 
from manual_sample import manual_sample
from auto_sample import auto_sample
from generate_outputs import gen_csv, gen_png, gen_wav

"""
The main file handles interaction between the sampling and output generation files. This file also handles
user interaction through the CLI.

Last modified by: 
Date last modified:
Changes made:
"""

def set_up():
    global ser
    global SAMPLE_RATE
    # List available ports
    devices = serial.tools.list_ports.comports()
    for dev in devices:
        print(dev.device)  # Use .device instead of [0] for better compatibility
    # Initialize serial port with proper settings
    ser = serial.Serial("COM14", 115200)
    SAMPLE_RATE = 5000

if (__name__ == "main"):
    set_up()
    

    