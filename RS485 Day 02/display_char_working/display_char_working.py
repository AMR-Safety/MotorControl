import serial
import time

# Open RS485 COM port
ser = serial.Serial('COM11', 9600, timeout=1)

# Character to send
char_to_send = '123456789'

# Send the character as a byte
ser.write(char_to_send.encode())
print("Sent:", char_to_send)

# Give Arduino time to process and echo back
time.sleep(0.1)

# Read 1 byte echoed back
echo = ser.read(1)

if echo:
    print("Echoed:", echo.decode())
else:
    print("No echo received")

ser.close()
