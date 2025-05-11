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
        sample_rate = 44000
        # List available ports
        devices = serial.tools.list_ports.comports()
        # for dev in devices:
        #     print(dev.device)  
        # Initialize serial port with proper settings
        ser = serial.Serial("COM4", 912600)
        mode = int(input("Menu\n1. Manual Recording Mode\n2. Distance Trigger Mode\nctrl+c to exit the program\nPlease enter the mode: "))
        start = time.time()
        received = 0

        if mode == 1:
            # ser.write(bytes([mode]))
            print("Mode 1 selected.")
            sec = int(input("Specify how long to record for: "))
            print(f"Recording for {sec} seconds...\n")
            # for _ in range(sec * sample_rate):
            #     buffer = ser.read(3)       # Read 1 byte 
            #     # print (byte[0])
            #     # received+=1
            #     byte[0] = (buffer[1]&0xF) | ((buffer[1]&0xF)>>4) | ((buffer[2]&0xF)>>8)
            #     byte[1] = (buffer[1]&0xF0) | ((buffer[1]&0xF0)>>4) | ((buffer[2]&0xF0)>>8)
            #     data.append(byte[0])     # append index 0
            #     data.append(byte[1])
            total_bytes = sec * sample_rate * 3 // 2  # Each 3 bytes = 2 samples
            raw = ser.read(total_bytes)
            print(raw)
            if len(raw) < total_bytes:
                print("Warning: Incomplete read.")

            for i in range(0, len(raw), 3):
                b0 = raw[i]
                b1 = raw[i+1]
                b2 = raw[i+2]

                # Unpack 12-bit samples for two channels
                ch0 = ((b2 & 0x0F) << 8) | ((b1 & 0x0F) << 4) | (b0 & 0x0F)
                ch1 = ((b2 & 0xF0) << 4) | (b1 & 0xF0) | ((b0 & 0xF0) >> 4)

                data.append(ch0)
                data.append(ch1)

            received = len(data)

        elif mode == 2:
            threshold = int(input("Please input your distance threshold (cm): "))
            ser.write(bytes([threshold]))
            print("Mode 2 selected. Waiting for proximity-triggered samples...")
            print(f"Recording will stop when distance > {threshold}cm.")

            while True:
                byte = ser.read(1)
                if byte:
                    data.append(byte[0])
                    # print(byte[0])
                    if byte[0] == 0:
                        break
                    
            data.pop(-1)
            duration = (len(data) - 1) / sample_rate  # Exclude the stop byte
            print(f"Recording finished. Duration: {duration} seconds, {len(data)} samples.")

        # elapsed = time.time() - start
        # rate = received / elapsed
        # print(f"Received {received} samples in {elapsed:.2f} seconds")
        # print(f"Estimated Sampling Rate: {rate:.2f} samples/sec")        
        ser.write(bytes([0]))
        ser.close()
        data = np.array(data)
        data = data - np.min(data)
        if np.max(data) > 0:
            data = data / np.max(data)
        data = (data * 255).astype(np.uint8)
        
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