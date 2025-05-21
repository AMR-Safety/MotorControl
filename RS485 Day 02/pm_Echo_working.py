import serial
import time

ser = serial.Serial('COM11', 9600, timeout=2)

message = "PROGRAMMING\n"
ser.write(message.encode())
print("Sent:", message.strip())

time.sleep(0.5)  # Short delay

# Read multiple lines if available
received = ""
start_time = time.time()
while time.time() - start_time < 5:  # Wait up to 2 seconds
    if ser.in_waiting:
        received += ser.read(ser.in_waiting).decode(errors='ignore')
    if "PROGRAMMING" in received:
        break

received = received.strip()
if received:
    print("Echoed:", received)
else:
    print("No echo received")

ser.close()
