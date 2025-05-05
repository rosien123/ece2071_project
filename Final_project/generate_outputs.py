import numpy as np
import wave
import matplotlib.pyplot as plt
import csv

"""
This file handles output generation only. The main file will call each function individually depending on
user input.

Each function should take in a numpy data array, generate the required output format and save it in the correct
file. The user can open this file on their device. Functions do not need to handle output viewing (for now).

Last modified by: 
Date last modified:
Changes made:
"""

def gen_wav(data: np, sample_rate: int):
    file_name = "testing.wav"
    with wave.open(file_name, 'wb') as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(1)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(data.tobytes())

def gen_png(data:np.ndarray, sample_rate: int):
    file_name = "testing.png"
    time = np.arange(len(data)) / int(sample_rate)

    plt.plot(time, data)

    plt.title('Audio Waveform')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Amplitude')

    plt.savefig(file_name)
    plt.close() #might change to show depending on if we want it open right away
    
def gen_csv(data:np.ndarray, sample_rate: int):
    file_name = "testing.csv"
    
    with open(file_name, mode='w', newline='') as file:
        csvwriter = csv.writer(file)
        csvwriter.writerow([f"Sample Rate: {sample_rate}"])
        #write all the values
        for d in data:
            csvwriter.writerow([d])

if __name__ == "__main__":
    # simulate some dummy data
    sample_rate = 5000  # 5k samples/sec
    raw_data = [10, 20, 30, 40, 50, 60, 70, 80]  # dummy list of integers
    data = np.array(raw_data)

    # normalize and scale like manual_sample would do
    data = (data - data.min()) / (data.max() - data.min())
    data = data * 255
    data = data.astype(np.uint8)

    # now test output generation
    gen_wav(data, sample_rate)
    gen_png(data, sample_rate)
    gen_csv(data, sample_rate)