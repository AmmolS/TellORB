import socket
from threading import Thread, Event

bufferSize = 1024

def stateReceiver():
    stateSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    stateSocket.bind(("", 8890))
    while True:

        # Receive response
        bytesAddressPair = stateSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)

        print(clientMsg)
        print(clientIP)

stateReceiverThread = Thread(target=stateReceiver)
stateReceiverThread.start()

serverAddressPort = ("192.168.10.1", 8889)
msg = "command"
bytesToSend = str.encode(msg)
# bufferSize  = 1024

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# UDPServerSocket.bind(("", 8890))
 
# Send data
UDPServerSocket.sendto(bytesToSend, serverAddressPort)

# Receive response
bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

message = bytesAddressPair[0]

address = bytesAddressPair[1]

clientMsg = "Message from Client:{}".format(message)
clientIP  = "Client IP Address:{}".format(address)

print(clientMsg)
print(clientIP)