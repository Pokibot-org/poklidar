import serial, socket, os, threading
import OPi.GPIO as GPIO
from time import sleep


def flashNucleo():
    os.system('mount /dev/stlinkv3_ /media/nucleo/')
    os.system('cp ./../firmware.bin /media/nucleo/')
    os.system('umount /media/nucleo')


class Motor:
    def __init__(self, pin_pwm, pins_dir):
        self.pin_pwm = pin_pwm
        self.pins_dir = pins_dir
        GPIO.setup(self.pin_pwm, GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
        GPIO.setup(self.pins_dir[0], GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
        GPIO.setup(self.pins_dir[1], GPIO.OUT, pull_up_down=GPIO.PUD_OFF)
        self.pwm = GPIO.PWM(self.pin_pwm, 800)
        self.stop()

    def go_forward(self, pwm = 100):
        # GPIO.output(self.pin_pwm, 1)
        self.pwm.start(pwm)
        GPIO.output(self.pins_dir[0], 1)
        GPIO.output(self.pins_dir[1], 0)
    def go_backward(self, pwm = 100):
        # GPIO.output(self.pin_pwm, 1)
        self.pwm.start(pwm)
        GPIO.output(self.pins_dir[0], 0)
        GPIO.output(self.pins_dir[1], 1)

    def stop(self):
        # GPIO.output(self.pin_pwm, 0)
        self.pwm.stop()
        GPIO.output(self.pins_dir[0], 0)
        GPIO.output(self.pins_dir[1], 0)



def updateMotor(motor, pwm):
    m = m_left if motor == "L" else m_right
    if pwm == 0:
        m.stop()
    elif pwm > 0:
        m.go_forward(pwm)
    elif pwm < 0:
        m.go_backward(-pwm)

global_stop = False
def startUART():
    global global_stop
    print("UART forward enabled")
    serverAddressPort   = ("192.168.0.18", 20002)
    bufferSize          = 512
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    ser = serial.Serial('/dev/ttyACM0', 460800, timeout=0.1)
    i = 0
    buffer = []
    message = ""
    while 1:
        try:
            v = [int(ser.read().hex(), 16)]
            # print(v[0])
            buffer += v
            # v = ser.read().hex()
            # message+=str(v)
            # print(v, message)
            i+=1
            if i >= 256 or v[0] == 10:
                # print("Send", i)
                bytesToSend = bytes(buffer)
                # bytesToSend = str.encode(message)
                UDPClientSocket.sendto(bytesToSend, serverAddressPort)
                i = 0
                buffer = []
                message = ""
        except Exception as e:
            # print("Exceptiooooooooooonnnnnnnnnnnn")
            # print(e)
            if i > 0:
                bytesToSend = bytes(buffer)
                # bytesToSend = str.encode(message)
                UDPClientSocket.sendto(bytesToSend, serverAddressPort)
                i = 0
                buffer = []
                message = ""
        if global_stop:
            break

# startUART()
# exit()

# Start UART forward
x = threading.Thread(target=startUART)
x.start()



# Hardware control for motors
GPIO.setboard(GPIO.PCPCPLUS)
GPIO.setmode(GPIO.BOARD)
m_left  = Motor(7, [5, 3])
m_right = Motor(15, [11, 13])



# Socket configuration
localIP     = "192.168.0.20"
localPort   = 20001
bufferSize  = 512
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Generate exception if communication is lost after 2 seconds
UDPServerSocket.settimeout(2)

UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")


while(True):
    try:
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        message = bytesAddressPair[0].split()
        address = bytesAddressPair[1]
        if len(message) == 2:
            if message[0] == b'ML':
                updateMotor("L", int(message[1]))
            if message[0] == b'MR':
                updateMotor("R", int(message[1]))
        if message[0] == b'STOP':
            print("Stopping...")
            global_stop = True
            updateMotor("L", 0)
            updateMotor("R", 0)
            GPIO.cleanup()
            exit()
        if message[0] == b'FLASH':
            print("Flashing nucleo...")
            flashNucleo()
    except Exception:
        # If communication is lost, reset all motors
        updateMotor("L", 0)
        updateMotor("R", 0)
