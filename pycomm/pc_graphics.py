
import sys, time, threading, math, socket
import random
from PIL import Image, ImageTk
from tkinter import Tk, Frame, Canvas, ALL, NW
import queue


q1 = queue.Queue()
global_stop = False

class Cons:

    BOARD_WIDTH = 1600
    BOARD_HEIGHT = 800
    DELAY = 100
    DOT_SIZE = 10
    MAX_RAND_POS = 27

class Lidar:
    POINTS_PER_TURN = 360
    ROTATE_CLOCKWISE = True
    PACKET_SIZE = 6
    commBuffer = []
    distances = [0]*POINTS_PER_TURN
    intensities = [0]*POINTS_PER_TURN
    yaw = 0
    yaw_data = [0]*250
    gyr_data = [0]*250
    liax_data = [0]*250
    liay_data = [0]*250
    yaw_guess = [0]*250
    imu_wp = 0

    def getPacketData(self, packet):
        (packet_valid, data) = packet
        data2 = []
        for i in range(6*3):
            data2+=[data[i*2+3] | (data[i*2+4]<<8)]
        angle_offset = (data[0]-160)*6
        angle = []
        distance = []
        intensity = []
        imu_data = []
        for i in range(6):
            angle+=[angle_offset + i]
            distance+=[data2[i*3+1]]
            intensity+=[data2[i*3+0]]
            imu_data+=[data2[i*3+2]]
        return (angle, distance, intensity, imu_data)

    def getPackets(self):
        while not q1.empty():
            self.commBuffer+=[q1.get()]
        while len(self.commBuffer) > 0 and self.commBuffer[0] != 0xFA:
            self.commBuffer = self.commBuffer[1:]
        packets = []
        while len(self.commBuffer) >= 42:
            packets += [self.commBuffer[:42]]
            self.commBuffer = self.commBuffer[42:]
        return packets

    def update(self):
        packets = self.getPackets()
        for p in packets:
            # TODO: check CRC
            (angle, distance, intensity, imu_data) = self.getPacketData((True, p[1:40]))
            if imu_data[0] == 0xFAAF:
                for i in range(5):
                    if imu_data[i] > 32768:
                        imu_data[i] = -65536+imu_data[i]
                # print(imu_data[1:5])
                self.yaw = imu_data[2]/16
                self.gyr_data[self.imu_wp] = imu_data[1]
                self.yaw_data[self.imu_wp] = imu_data[2]/16
                self.liax_data[self.imu_wp] = imu_data[3]
                self.liay_data[self.imu_wp] = imu_data[4]

                w = 50
                j0 = (self.imu_wp-w)%250
                self.yaw_guess[j0] = self.yaw_data[j0]
                for i in range(w):
                    j0 = (self.imu_wp-w+i)%250
                    j1 = (self.imu_wp-w+1+i)%250
                    v = self.yaw_guess[j0]-(self.gyr_data[j0]/8)*0.1
                    v = v%360
                    self.yaw_guess[j1] = v
                self.imu_wp = (self.imu_wp+1)%250
            for i in range(len(angle)):
                if angle[i] >= 0 and angle[i] < len(self.distances):
                    self.distances[angle[i]] = distance[i]
                    self.intensities[angle[i]] = intensity[i]

class Board(Canvas):
    score = 0
    lidar = Lidar()
    lidarLines = []


    def __init__(self):
        super().__init__(width=Cons.BOARD_WIDTH, height=Cons.BOARD_HEIGHT,
            background="black", highlightthickness=0)

        self.initGame()
        self.pack()


    def initGame(self):
        self.createObjects()
        self.after(Cons.DELAY, self.onTimer)

    def createObjects(self):
        self.create_text(30, 10, text="Score: {0}".format(self.score),
        tag="score", fill="white")

    def onTimer(self):
        self.drawScore()
        self.after(Cons.DELAY, self.onTimer)

    def drawGraph(self, data, x0, y0, dx, dy, color="#fb0"):
        for i in range(len(data)-1):
            l_id = self.create_line(i*dx+x0, data[i]*dy+y0, (i+1)*dx+x0, data[i+1]*dy+y0, fill=color)

    def drawScore(self):
        score = self.find_withtag("score")
        self.lidar.update()


        # offset = -int(self.lidar.yaw*self.lidar.POINTS_PER_TURN/360)
        offset = 0
        lenght = self.lidar.POINTS_PER_TURN
        self.delete("all")

        self.drawGraph(self.lidar.yaw_data, 0, 0, 2, 0.5, "#f00")
        self.drawGraph(self.lidar.gyr_data, 0, 90, 2, 0.05, "#00f")
        self.drawGraph(self.lidar.yaw_guess, 0, 0, 2, 0.5, "#0f0")
        self.drawGraph(self.lidar.liax_data, 0, 90+180, 2, 0.5, "#f00")
        self.drawGraph(self.lidar.liay_data, 0, 90+180, 2, 0.5, "#00f")

        self.lidarLines = []
        distances = [self.lidar.distances[(i+offset)%lenght] for i in range(lenght)]
        for i in range(self.lidar.POINTS_PER_TURN):
            angle = 2*math.pi*i/self.lidar.POINTS_PER_TURN
            angle+=-math.pi/2
            dx = math.cos(angle)*distances[i]*0.1
            dy = math.sin(angle)*distances[i]*0.1
            if abs(dx) < 400 and abs(dy) < 400:
                l_id = self.create_line(400, 400, 400+dx, 400+dy, fill="#fb0")
                self.lidarLines += [l_id]
        intensities = [self.lidar.intensities[(i+offset)%lenght] for i in range(lenght)]
        for i in range(self.lidar.POINTS_PER_TURN):
            angle = 2*math.pi*i/self.lidar.POINTS_PER_TURN
            angle+=-math.pi/2
            dx = math.cos(angle)*intensities[i]*0.05
            dy = math.sin(angle)*intensities[i]*0.05
            if abs(dx) < 400 and abs(dy) < 400:
                l_id = self.create_line(1200, 400, 1200+dx, 400+dy, fill="#0bf")
                self.lidarLines += [l_id]
        # self.itemconfigure(score, text="Score: {0}".format(self.score))





class SuperFrame(Frame):

    def __init__(self):
        super().__init__()

        self.master.title('Poklidar')
        self.board = Board()
        self.pack()


def threadLidar():
    global global_stop
    # i = 0
    # while not global_stop:
    #     time.sleep(1.0)
    #     q1.put(i)
    #     i+=1
    # Socket configuration
    localIP     = "192.168.0.18"
    localPort   = 20002
    bufferSize  = 1024
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Generate exception if communication is lost after 2 seconds
    # UDPServerSocket.settimeout(2)

    UDPServerSocket.bind((localIP, localPort))
    print("UDP server up and listening")


    while not global_stop:
        try:
            bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
            message = bytesAddressPair[0]#.decode()
            address = bytesAddressPair[1]
            for j in message:
                i = int(j)
                q1.put(i)
        except Exception:
            print("", end="")
            # Oops


def main():

    # threadLidar()
    # exit()

    x = threading.Thread(target=threadLidar)
    x.start()

    root = Tk()
    nib = SuperFrame()
    root.mainloop()


if __name__ == '__main__':
    main()
