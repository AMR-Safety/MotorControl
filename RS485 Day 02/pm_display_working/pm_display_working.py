import serial
import time

# Replace 'COMx' with your actual RS485 USB adapter port (check Device Manager)
ser = serial.Serial('COM11', 9600, timeout=1)

# Character to send
char_to_send = '123456789'

# Send the character
ser.write(char_to_send.encode())
print("Sent:", char_to_send)

# Allow time for Arduino to echo back
time.sleep(0.1)

# Read the echoed byte
echo = ser.read(1)

if echo:
    print("Echoed:", echo.decode())
else:
    print("No echo received")

ser.close()
