import socket

# Socket configuration
localIP     = "192.168.0.18"
localPort   = 20002
bufferSize  = 1024
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Generate exception if communication is lost after 2 seconds
# UDPServerSocket.settimeout(2)

UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")


while(True):
    try:
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        message = bytesAddressPair[0].decode()
        address = bytesAddressPair[1]
        print(message, end="")
    except Exception:
        print("", end="")
        # Oops
