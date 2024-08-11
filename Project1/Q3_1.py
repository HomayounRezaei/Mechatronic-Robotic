import serial


ser = serial.Serial('COM12', 115200)  

while True:
    
    data = ser.readline().decode().strip()
    if data:
        
        print(data)
