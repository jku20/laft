import serial
import time

# USER SETTINGS Jeremy Change for Weird Linux Stuff
PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 0.5


ser = serial.Serial(PORT, baudrate=BAUD)

S1 = ["--    ", "    --", "------", "      "]
S2 = ["  \\   ", "   /  ", "      ", "      "]
S3 = ["   \\  ", "  /   ", "      ", "      "]
S4 = ["    --", "--    ", "      ", "------"]


def pretty_print_waveform(data):
    for S in [S1, S2, S3, S4]:
        for i in range(len(data)):
            if i == len(data) - 1:
                if data[i] == 0:
                    print(S[3][:3], end="")
                else:
                    print(S[2][:3], end="")
            else:
                d1, d2 = data[i], data[i + 1]
                if d1 == d2:
                    if d1 == 0:
                        print(S[3], end="")
                    else:
                        print(S[2], end="")
                else:
                    if d1 == 0:
                        print(S[1], end="")
                    else:
                        print(S[0], end="")
        print()


def get_channel_n(data, n):
    out = []
    for i in range(len(data) // 4):
        if n < 8:
            out.append((data[4 * i] & (1 << n)) >> n)
        else:
            out.append((data[4 * i + 1] & (1 << n)) >> n)
    return out


def send(cmd_bytes):
    """
    Send raw command bytes to SUMP device.
    """
    ser.write(cmd_bytes)
    ser.flush()
    time.sleep(0.1)


print("Connected to SUMP device.")

# 0x00 = RESET
print("Sending 0x00 = RESET")
send(b"\x00")

# 0x02 = ID
print("Requesting DEVICE ID...")
send(b"\x02")
data = ser.read(4)
print(data[::-1])


# Trigger configuration example
print("\nConfiguring Stage 0")

# Stage 0 mask
send(bytes([0xC0, 0x80, 0x00, 0x00, 0x00]))

# Stage 0 value
send(bytes([0xC1, 0x80, 0x00, 0x00, 0xA5]))

# Stage 0 config
send(bytes([0xC2, 0x00, 0x00, 0x00, 0x01]))

# Stage 1 mask
send(bytes([0xC4, 0x00, 0x00, 0x00, 0x00]))

# Stage 2 mask
send(bytes([0xC8, 0x00, 0x00, 0x00, 0x00]))

# Stage 3 mask
send(bytes([0xCC, 0x00, 0x00, 0x00, 0x00]))


# Set sample count 0x81, request 4096 samples
sample_count = 16
cnt_bytes = sample_count.to_bytes(2, "little")

print(f"Setting sample count to {sample_count}...")
send(b"\x81" + cnt_bytes + cnt_bytes)

# print("\nSetting divider to 4...")
# send(b"\x80\x04\x00\x00\x04")

# Set flags

# send(b"\x82\xff\xff\xff\x7f")

# Arm the capture again

print("\nRUN capture...")
send(b"\x01")

data = ser.read(sample_count * 4)
c0 = get_channel_n(data, 0)
pretty_print_waveform(c0)

print("\nTrigger on falling edge")
# Stage 1 mask
send(bytes([0xC4, 0x80, 0x00, 0x00, 0x00]))

# Stage 1 value
send(bytes([0xC5, 0x00, 0x00, 0x00, 0x00]))

# Stage 1 config
send(bytes([0xC6, 0x00, 0x00, 0x00, 0x01]))

print("\nRUN capture...")
send(b"\x01")

data = ser.read(sample_count * 4)
c0 = get_channel_n(data, 0)
pretty_print_waveform(c0)

print("\nDone.")
ser.close()
