import sys
import serial
from tkinter import *

# Serial Config
serialPort = "/dev/ttyACM0"
baudrate = 115200

try:
    s = serial.Serial(serialPort, baudrate)
except Exception as e:
    print("\n", e)
    print(f"> No device found on {serialPort}, probably")
    sys.exit(1)

def send_command(val):
    """Send the new slider values to the Arduino as a comma-separated string"""
    msg = "{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
        scale1.get()+90, scale2.get()+90, scale3.get()+90, scale4.get()+90, scale5.get()+90, 190-scale6.get()
    )
    try:
        s.write(msg.encode())
    except Exception as e:
        print("> Serial write failed:", e)

root = Tk()
root.geometry("500x700")
root.title("Servo Control GUI")


scale1 = Scale(root, label="Joint 1", from_=-100, to=100, orient=HORIZONTAL, length=400, command=send_command)
scale1.pack()
scale1.set(0)
scale2 = Scale(root, label="Joint 2", from_=-90, to=105, orient=HORIZONTAL, length=400, command=send_command)
scale2.pack()
scale2.set(0)
scale3 = Scale(root, label="Joint 3", from_=-100, to=100, orient=HORIZONTAL, length=400, command=send_command)
scale3.pack()
scale3.set(0)
scale4 = Scale(root, label="Joint 4", from_=-100, to=100, orient=HORIZONTAL, length=400, command=send_command)
scale4.pack()
scale4.set(0)
scale5 = Scale(root, label="Joint 5", from_=-90, to=90, orient=HORIZONTAL, length=400, command=send_command)
scale5.pack()
scale5.set(0)
scale6 = Scale(root, label="Joint 6", from_=0, to=100, orient=HORIZONTAL, length=400, command=send_command)
scale6.pack()
scale6.set(0)

root.mainloop()

print("Closing down")
s.close()
