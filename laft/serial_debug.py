import serial
import time

# USER SETTINGS Jeremy Change for Weird Linux Stuff
PORT = 'COM7'
BAUD = 115200      
TIMEOUT = 0.5


ser = serial.Serial(PORT, baudrate=BAUD, timeout=TIMEOUT)

def send(cmd_bytes):
    """
    Send raw command bytes to SUMP device.
    """
    ser.write(cmd_bytes)
    ser.flush()
    time.sleep(0.05)
    data = ser.read(1024)
    print(f"Sent: {cmd_bytes.hex()} -> Received: {data.hex()} ({data})")
    return data

print("Connected to SUMP device.")
print("Testing basic SUMP commands...\n")


# 0x00 = RESET
print("Sending 0x00 = RESET")
send(b'\x00')

# 0x01 = RUN
print("Sending RUN (start capture)...")
send(b'\x01')


# 0x02 = ID
print("Requesting DEVICE ID...")
send(b'\x02')


# Trigger configuration example
print("\nConfiguring Stage 0: mask=0xFF, value=0xA5")

# Stage 0 mask 
send(bytes([0xC0, 0x00, 0x00, 0x00, 0xFF]))

# Stage 0 value 
send(bytes([0xC1, 0x00, 0x00, 0x00, 0xA5]))

# Stage 0 config 
send(bytes([0xC2, 0x00, 0x00, 0x00, 0x01]))


# Set sample count 0x81, request 4096 samples
sample_count = 4096
cnt_bytes = sample_count.to_bytes(3, 'big')

print("\nSetting sample count to 4096...")
send(b'\x81' + cnt_bytes)

# Set sample rate divisor, 0x82 + 2 bytes (big endian), samplerate = clock / (div + 1)

print("\nSetting divider to 4...")
send(b'\x80\x00\x04\x00\x04') 

# Set flags

send(b'\x82\xFF\xFF\xFF\x7F')

# Arm the capture again

print("\nRUN capture...")
send(b'\x01')

print("\nDone.")
ser.close()
