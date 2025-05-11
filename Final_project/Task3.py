import numpy as np
import wave
import serial
import serial.tools.list_ports
import time
import matplotlib.pyplot as plt
import csv

while True:
    try: 
        data = []
        sample_rate = 22000

        # List available ports
        devices = serial.tools.list_ports.comports()
        # for dev in devices:
        #     print(dev.device)  
        # Initialize serial port with proper settings
        ser = serial.Serial("COM4", 352800)
        received = 0

        mode = int(input("Menu\n1. Manual Recording Mode\n2. Distance Trigger Mode\nctrl+c to exit the program\nPlease enter the mode: "))

        if mode == 1:
            ser.write(bytes([mode]))
            print("Mode 1 selected.")
            sec = int(input("Specify how long to record for: "))
            print(f"Recording for {sec} second...\n")
            for _ in range(sec * sample_rate):
                byte = ser.read(1)       # Read 1 byte 
                received+=1
                # print(byte[0])
                data.append(byte[0])     # append index 0

        elif mode == 2:
            threshold = int(input("Please input your distance threshold (cm): "))
            ser.write(bytes([threshold]))
            print("Mode 2 selected. Waiting for proximity-triggered samples...")
            print(f"Recording will stop when distance > {threshold}cm.")

            while True:
                byte = ser.read(1)
                received+=1
                if byte:
                    data.append(byte[0])
                    # print(byte[0])
                    if byte[0] == 0:
                        break
                    
            data.pop(-1)
            duration = (len(data) - 1) / sample_rate  # Exclude the stop byte
            print(f"Recording finished. Duration: {duration} seconds, {len(data)} samples.")

        rate = received/sec
        print(f"Received {received} samples in {rate:.2f} hertz")      
        ser.write(bytes([0]))
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