import socket
import tkinter as tk
import threading, time, os


class KeyTracker():
    key = ''
    last_press_time = 0
    last_release_time = 0

    def track(self, key):
        self.key = key

    def is_pressed(self):
        return time.time() - self.last_press_time < .1

    def report_key_press(self, event):
        # if not self.is_pressed():
        on_key_press(event)
        self.last_press_time = time.time()

    def report_key_release(self, event):
        timer = threading.Timer(.1, self.report_key_release_callback, args=[event])
        timer.start()

    def report_key_release_callback(self, event):
        if not self.is_pressed():
            on_key_release(event)
        self.last_release_time = time.time()


def on_key_release(event):
    # print("Released", event.keysym)
    motor = "L"
    pwm = 0
    if event.keysym == 'a' or event.keysym == 'q':
        motor = 'L'
        pwm = 0
    if event.keysym == 'e' or event.keysym == 'd':
        motor = 'R'
        pwm = 0
    updateMotor(UDPClientSocket, serverAddressPort, motor, pwm)

def on_key_press(event):
    # print("Pressed", event.keysym)
    motor = "L"
    pwm = 0
    if event.keysym == 'a':
        motor = 'L'
        pwm = 50
    if event.keysym == 'e':
        motor = 'R'
        pwm = 50
    if event.keysym == 'q':
        motor = 'L'
        pwm = -50
    if event.keysym == 'd':
        motor = 'R'
        pwm = -50
    updateMotor(UDPClientSocket, serverAddressPort, motor, pwm)

def stopControl(event, socket, address):
    message = "STOP"
    bytesToSend = str.encode(message)
    socket.sendto(bytesToSend, address)
    root.destroy()

def flashNucleo(event, socket, address):
    message = "FLASH"
    bytesToSend = str.encode(message)
    socket.sendto(bytesToSend, address)


def updateMotor(socket, address, motor, pwm):
    message = "M"+motor+" "+str(pwm)
    bytesToSend = str.encode(message)
    socket.sendto(bytesToSend, address)


serverAddressPort   = ("192.168.0.20", 20001)
bufferSize          = 512

UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)


root = tk.Tk()

key_trackers = []
keys = ['a', 'q', 'e', 'd']
for i in range(4):
    key_trackers+=[KeyTracker()]
    root.bind('<KeyPress-'+keys[i]+'>', key_trackers[i].report_key_press)
    root.bind('<KeyRelease-'+keys[i]+'>', key_trackers[i].report_key_release)
root.bind('<KeyPress-Delete>', lambda event: stopControl(event, UDPClientSocket, serverAddressPort))
root.bind('<KeyPress-Insert>', lambda event: flashNucleo(event, UDPClientSocket, serverAddressPort))

root.mainloop()
