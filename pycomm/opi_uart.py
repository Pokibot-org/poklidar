import serial, socket

# 187500
# 214285 (230400)
# 250000
# 250000
# 300000
# 375000
# 500000

serverAddressPort   = ("192.168.0.18", 20002)
bufferSize          = 512

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


ser = serial.Serial('/dev/ttyACM0', 460800, timeout=0.1)

i = 0
buffer = []
while 1:
    try:
        v = int(ser.read().hex(), 16)
        buffer += [v]
        i+=1
        if i >= 8:
            UDPClientSocket.sendto(bytes(buffer), serverAddressPort)
            i = 0
            buffer = []
    except Exception:
        if i > 0:
            UDPClientSocket.sendto(bytes(buffer), serverAddressPort)
            i = 0
            buffer = []
