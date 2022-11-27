from time import sleep
import tellopy

# Parse some of the interesting info from the tello log stream
def parseLog(data):
    pos = 0 

    # A packet can contain more than one record.
    while pos < data.Length-2: # -2 for CRC bytes at end of packet.
        if data[pos] != 'U': # Check magic byte
            print("PARSE ERROR!!!")
            break
        len = data[pos + 1]
        if data[pos + 2] != 0: # Should always be zero (so far)
            print("SIZE OVERFLOW!!!")
            break
        crc = data[pos + 3]
        # id = BitConverter.ToUInt16(data, pos + 4)
        # xorBuf = new byte[256]
        # byte xorValue = data[pos + 6]
        # switch (id)
        # {
        #     case 0x1d://29 new_mvo
        #         for (i = 0; i < len; i++)//Decrypt payload.
        #             xorBuf[i] = (byte)(data[pos + i] ^ xorValue);
        #         var index = 10;//start of the velocity and pos data.
        #         var observationCount = BitConverter.ToUInt16(xorBuf, index); index += 2;
        #         velX = BitConverter.ToInt16(xorBuf, index); index += 2;
        #         velY = BitConverter.ToInt16(xorBuf, index); index += 2;
        #         velZ = BitConverter.ToInt16(xorBuf, index); index += 2;
        #         posX = BitConverter.ToSingle(xorBuf, index); index += 4;
        #         posY = BitConverter.ToSingle(xorBuf, index); index += 4;
        #         posZ = BitConverter.ToSingle(xorBuf, index); index += 4;
        #         posUncertainty = BitConverter.ToSingle(xorBuf, index)*10000.0f; index += 4;
        #         //Console.WriteLine(observationCount + " " + posX + " " + posY + " " + posZ);
        #         break;
        #     case 0x0800://2048 imu
        #         for (var i = 0; i < len; i++)//Decrypt payload.
        #             xorBuf[i] = (byte)(data[pos + i] ^ xorValue);
        #         var index2 = 10 + 48;//44 is the start of the quat data.
        #         quatW = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         quatX = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         quatY = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         quatZ = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         //Console.WriteLine("qx:" + qX + " qy:" + qY+ "qz:" + qZ);

        #         //var eular = toEuler(quatX, quatY, quatZ, quatW);
        #         //Console.WriteLine(" Pitch:"+eular[0] * (180 / 3.141592) + " Roll:" + eular[1] * (180 / 3.141592) + " Yaw:" + eular[2] * (180 / 3.141592));

        #         index2 = 10 + 76;//Start of relative velocity
        #         velN = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         velE = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         velD = BitConverter.ToSingle(xorBuf, index2); index2 += 4;
        #         //Console.WriteLine(vN + " " + vE + " " + vD);

        #         break;

        # }
        pos += len



def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)
    elif event is drone.EVENT_LOG:
        print(data)


drone = tellopy.Tello()
try:
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.subscribe(drone.EVENT_LOG, handler)

    drone.connect()
    drone.wait_for_connection(60.0)
    # drone.takeoff()
    sleep(5)
    # drone.down(50)
    sleep(5)
    # drone.land()
    sleep(5)
except Exception as ex:
    print(ex)
finally:
    drone.quit()

