import numpy as np
import wave
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
import csv

data = []
sample_rate = 44100

def decode_12bit_data(data_bytes):
    values = []
    for i in range(0, len(data_bytes), 2):
        if i+1 < len(data_bytes):
            value = (data_bytes[i] << 4) | (data_bytes[i+1] & 0x0F)
            values.append(value)
    return values

while True:
    try:
        # List available ports
        devices = serial.tools.list_ports.comports()
        for dev in devices:
            print(f"Found port: {dev.device}")

        # Initialize serial port with proper settings
        ser = serial.Serial("COM4", 760000)
        chunk = 2048

        mode = int(input("Menu\n1. Manual Recording Mode\n2. Distance Trigger Mode\nctrl+c to exit the program\nPlease enter the mode: "))

        if mode == 1:
            ser.write(bytes([mode]))
            print("Mode 1 selected.")
            sec = int(input("Specify how long to record for: "))
            print(f"Recording for {sec} second...\n")
            total_chunks = int(sec * sample_rate // (chunk/2)) + 2

            for i in range(total_chunks):    
                # Read chunk of data
                byte = ser.read(chunk)
                if byte:
                    # Decode the raw bytes to 12-bit values
                    values = decode_12bit_data(byte)
                    
                    # Keep raw data for comparison
                    data.extend(values)
                    #print(values)
        
        ser.close()
        data = np.array(data)
        data = (data - data.min()) / data.max()
        data = data * 255
        data = data.astype(np.uint8)

        while True:
            try:
                print("Data recorded! Please choose which file to open...")
                filetype = int(input("1. wav\n2. png\n3. csv\ncrtl+c to return to menu\nFile type: "))

                if filetype == 1:
                    file_name = "testing.wav"
                    with wave.open(file_name, 'wb') as wav_file:
                        wav_file.setnchannels(1)
                        wav_file.setsampwidth(1)
                        wav_file.setframerate(sample_rate)
                        wav_file.writeframes(data.tobytes())
                    print("wav file successfully open! Please check the explorer.\n")

                elif filetype == 2:
                    file_name = "testing.png"
                    time = np.arange(len(data)) / int(sample_rate)

                    plt.plot(time, data)

                    plt.title('Audio Waveform')
                    plt.xlabel('Time (seconds)')
                    plt.ylabel('Amplitude')

                    plt.savefig(file_name)
                    plt.close() #might change to show depending on if we want it open right away
                    print("png file successfully open! Please check the explorer.\n")

                elif filetype == 3:
                    file_name = "testing.csv"
                    
                    with open(file_name, mode='w', newline='') as file:
                        csvwriter = csv.writer(file)
                        csvwriter.writerow([f"Sample Rate: {sample_rate}"])
                        #write all the values
                        for d in data:
                            csvwriter.writerow([d])
                    print("csv file successfully open! Please check the explorer.\n")

            except KeyboardInterrupt:
                print("Going back to menu...\n")
                break
    
    except KeyboardInterrupt:
        print("\nSystem ending...Byebye...")
        break              

