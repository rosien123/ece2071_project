import numpy as np
import serial
import serial.tools.list_ports
import time 

"""
This file handles distance-triggered sampling from the processing STM (Task 2). The auto_sample function should
interact with the processing STM to generate a numpy array of the data. This data should be returned.

Last modified by: 
Date last modified:
Changes made:
"""

def auto_sample(ser, sample_rate):
    # auto sampling code here
    data = []
    data = np.array([])

    print("Auto sample!")

    return data
