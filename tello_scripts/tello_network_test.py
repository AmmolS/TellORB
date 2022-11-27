import socket, logging
from threading import Thread, Event
from datetime import datetime

def uint16(val0, val1):
    return (val0 & 0xff) | ((val1 & 0xff) << 8)

log = logging.Logger('Tello')

bufferSize = 2000

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDPServerSocket.bind(('', 8889))
# UDPServerSocket.settimeout(2.0)

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

        print("Get packet at port 8890")
        print(clientMsg)
        print(clientIP)
        print(datetime.now())

def extraReceiver():
    while True:

        # Receive response
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]

        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)
        

        print("Got packet at port 8889")
        print(clientMsg)
        print(clientIP)
        print(datetime.now())

        if isinstance(message, str):
            message = bytearray([x for x in message])
            print(message)
        else:
            print("Not string")


        port = 9617
        port0 = (int(port/1000) % 10) << 4 | (int(port/100) % 10)
        port1 = (int(port/10) % 10) << 4 | (int(port/1) % 10)
        buf = 'conn_req:%c%c' % (chr(port0), chr(port1))
        print(port0)
        print(port1)
        print(buf)


        if str(message[0:9]) == 'conn_ack:' or message[0:9] == b'conn_ack:':
            # log.info('connected. (port=%2x%2x)' % (message[9], message[10]))
            print('connected. (port=%2x%2x)' % (message[9], message[10]))
            # log.debug('    %s' % byte_to_hexstring(message))
            # if self.video_enabled:
            #     self.__send_exposure()
            #     self.__send_video_encoder_rate()
            #     self.__send_start_video()
            # self.__publish(self.__EVENT_CONN_ACK, message)

            print("returning true?")
        
        cmd = uint16(message[5], message[6])
        print(cmd)

        
        

# stateReceiverThread = Thread(target=stateReceiver)
# stateReceiverThread.start()

extraReceiverThread = Thread(target=extraReceiver)
extraReceiverThread.start()

serverAddressPort = ("192.168.10.1", 8889)
msg = "command"
bytesToSend = str.encode(msg)
# bufferSize  = 1024

# Create a datagram socket

# UDPServerSocket.bind(("", 8890))
 
# Send data
UDPServerSocket.sendto(bytesToSend, serverAddressPort)
