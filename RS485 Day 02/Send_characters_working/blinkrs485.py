import serial
ser = serial.Serial('COM11', 9600, timeout=1)
ser.write(b'G')
print("Echo:", ser.read())  # Should print: b'Z'
