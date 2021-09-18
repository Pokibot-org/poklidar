import serial
import time


class HLS_LFCD2:
    def __init__(self):
        self.ser = None
    def open(self, peripheral = '/dev/ttyUSB0'):
        self.ser = serial.Serial(peripheral, 230400)

    def start(self):
        self.ser.write(b'b')
    def stop(self):
        self.ser.write(b'e')

    def get_lidar_packet(self):
        d = 0
        while d != 0xFA:
            d = int(self.ser.read().hex(), 16)
        data = []
        checksum = d
        for i in range(41):
            d = int(self.ser.read().hex(), 16)
            checksum+=d
            data+=[d]
        checksum = (checksum+1)&0xFF
        packet_valid = checksum == data[39] and checksum == data[40]
        if not packet_valid:
            print("Checksum error", hex(checksum), hex(data[-1]), hex(data[-2]))
        return (packet_valid, data[:39])



    def get_packet_data(self, packet):
        (packet_valid, data) = packet
        data2 = []
        for i in range(6*3):
            data2+=[data[i*2+1] | (data[i*2+2]<<8)]
        angle_offset = (data[0]-160)*6
        angle = []
        distance = []
        intensity = []
        for i in range(6):
            angle+=[angle_offset + i]
            distance+=[data2[i*3+2]]
            intensity+=[data2[i*3+1]]
        return (angle, distance, intensity)

    def get_one_scan(self):
        data = [0]*360
        angle = [0]
        while angle[-1] != 359:
            packet = self.get_lidar_packet()
            packet = self.get_packet_data(packet)
            (angle, distance, intensity) = packet
            for i in range(6):
                data[angle[i]] = distance[i]
        return data


#
# startTime = time.time()
#
lidar = HLS_LFCD2()
lidar.open()
lidar.start()

print(lidar.get_one_scan())

lidar.stop()
