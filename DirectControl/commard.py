
import serial
import math
import csv
import struct
import time
import tkinter as tk

arduinoComPort = "COM9"
baudRate = 9600
 
serialPort = serial.Serial(arduinoComPort, baudRate, timeout=1) #Opens the Serial Port

forward = False
root = tk.Tk()

def left_forward(event):
    serialPort.write(struct.pack('>B', 1))

def right_forward(event):
    serialPort.write(struct.pack('>B', 2))

def left_backward(event):
    serialPort.write(struct.pack('>B', 3))

def right_backward(event):
    serialPort.write(struct.pack('>B', 4))

def stop(event):
    serialPort.write(b'0') ##sends '1' to serial 

def speedup(event):
    serialPort.write(struct.pack('>B', 5))

def slowdown(event):
    serialPort.write(struct.pack('>B', 6))


frame = tk.Frame(root)
frame.pack()

root.geometry("500x500") #You want the size of the app to be 500x500
root.resizable(0, 0) #Don't allow resizing in the x or y direction

button = tk.Button(frame, 
                   text="QUIT", 
                   fg="red",
                   command = quit,
                   font = "Fixedsys")
button.grid(row = 7, column=2)
button.config(width = 10, height = 3)
# button.Font(family = )


left_fwd = tk.Button(frame,
                  text="Left Wheel Forward",
                  font = "Fixedsys",
                  bg = "pale turquoise")

left_fwd.grid(row = 2, column=1)
left_fwd.bind('<KeyPress-e>', left_forward)
left_fwd.bind('<Enter>', left_forward)
left_fwd.bind('<KeyRelease-e>', stop)
left_fwd.bind('<Leave>', stop)

left_fwd.config(width = 20,height = 5)



right_fwd = tk.Button(frame,
                   text="Right Wheel Forward",
                  font = "Fixedsys",
                  bg = "DarkSeaGreen1")

right_fwd.grid(row = 2,column=3)

right_fwd.bind('<KeyPress-i>', right_forward)
right_fwd.bind('<Enter>', right_forward)
right_fwd.bind('<KeyRelease-i>', stop)
right_fwd.bind('<Leave>', stop)

right_fwd.config(width = 20,height = 5)



left_bwd = tk.Button(frame,
                   text="Reverse",
                  font = "Fixedsys",
                  bg = "light goldenrod")

left_bwd.grid(row = 3,column=1)
left_bwd.bind('<KeyPress-s>', left_backward)
left_bwd.bind('<KeyRelease-s>', stop)
left_bwd.config(width = 20,height = 5)


right_bwd = tk.Button(frame,
                   text="Reverse",
                  font = "Fixedsys",
                  bg = "misty rose")

right_bwd.grid(row = 3,column=3)
right_bwd.bind('KeyPress-l', right_backward)
right_bwd.bind('<Enter>', right_backward)
right_bwd.bind('<KeyRelease-l>', stop)
right_bwd.bind('<Leave>', stop)

right_bwd.config(width = 20,height = 5)


Speed = tk.Button(frame,
                   text="speed up",
                  font = "Fixedsys")

Speed.grid(row = 4,column=5)
Speed.bind('<Button-1>', speedup)
Speed.bind('<ButtonRelease-1>', stop)
Speed.bind('<KeyPress-z>', speedup)


slow = tk.Button(frame,
                   text="slowdown",
                  font = "Fixedsys")

slow.grid(row = 5,column=5)
slow.bind('<Button-1>', slowdown)
slow.bind('<ButtonRelease-1>', stop)
slow.bind('<KeyPress-a>', slowdown)
slow.bind('<KeyRelease-a>', stop)

root.mainloop()