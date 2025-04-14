import serial
import serial.tools.list_ports
import time

message = "Hello"

# Checksum function
def checksum_operation(string):
    checksum = ord(string[0])
    for char in string[1:]:
        checksum ^= ord(char)
    return checksum

# List available ports
devices = serial.tools.list_ports.comports()
for dev in devices:
    print(dev.device)  # Use .device instead of [0] for better compatibility

# Initialize serial port with proper settings
ser = serial.Serial("/dev/tty.usbmodem1103", 115200)

# Prepare message (pad to 80 bytes to match STM32 expectation)
checksum = checksum_operation(message)
string = message + ',' + str(checksum)
print(f"Original message: {string}")

# Encode and pad to 80 bytes with null bytes
send_data = string.encode('ascii').ljust(80, b'\x00')[:80]
print(f"Sending: {send_data} (length: {len(send_data)} bytes)")

# Send data
ser.write(send_data)

while True:
    # Read response (expecting 80 bytes)
    received_data = ser.read(80)
    
    if received_data:  # If data was actually received
        try:
            # Try to decode as ASCII, but show raw bytes if it fails
            decoded = received_data.decode('ascii').strip('\x00')
            print("12314")
            print(f"Received: {decoded} (raw: {received_data})")
        except UnicodeDecodeError:
            print(f"Received non-ASCII data: {received_data}")