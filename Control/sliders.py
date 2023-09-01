import sys

enableAutoInstall = False

if enableAutoInstall:
    import pip
    pip.main(['install', 'tk'])
    pip.main(['install', 'pyserial'])


import serial
from tkinter import *

# Try to open up the serial port
# Change these if needed
serialPort = "COM5"
baudrate = 115200

# check if the port exists
# and tells you if it doesn't
try:
    s = serial.Serial(serialPort, baudrate)
except Exception as e:
    print("\n", e)
    print(f">No device found on {serialPort} probably")

    sys.exit(69)


def sup(val):
    """Send the new slider values to the Arduino as a comma-separated string
    Args:
        val ([str]): The new slider positions as a comma-separated string
    """
    # Add semicolon after the value
    msg = val + ";"

    # Encode the message
    encodedMessage = msg.encode()

    # Send the encoded message
    s.write(encodedMessage)

# Create the tkinter window
root = Tk()
root.geometry("500x700")
# Load the background image
background_image = PhotoImage(file="C:\\Users\\FADI\\Desktop\\robotics\\ArthRobot\\ArthRobot_Control\\GUI\\arthrobot_logo.png")

# Create a Label to display the background image
background_label = Label(root, image=background_image)
background_label.place(x=0, y=0, relwidth=1, relheight=1, in_=root)  # Cover the entire window

# Create four sliders, each controlling one servo, and send their values as a comma-separated string
scale1 = Scale(root,
               variable=DoubleVar(value=-3),
               width=10,
               length=400,
               orient=HORIZONTAL,
               from_=-90,
               to=90,
               command=lambda val: sup(f"{val},{scale2.get()},{scale3.get()},{scale4.get()}, {scale5.get()}"))
scale1.pack()

scale2 = Scale(root,
               variable=DoubleVar(value=-58),
               width=10,
               length=400,
               orient=HORIZONTAL,
               from_=-90,
               to=90,
               command=lambda val: sup(f"{scale1.get()},{val},{scale3.get()},{scale4.get()}, {scale5.get()}"))
scale2.pack()

scale3 = Scale(root,
               variable=DoubleVar(value=90),
               width=10,
               length=400,
               orient=HORIZONTAL,
               from_=-90,
               to=90,
               command=lambda val: sup(f"{scale1.get()},{scale2.get()},{val},{scale4.get()}, {scale5.get()}"))
scale3.pack()

scale4 = Scale(root,
               variable=DoubleVar(value=47),
               width=10,
               length=400,
               orient=HORIZONTAL,
               from_=-90,
               to=90,
               command=lambda val: sup(f"{scale1.get()},{scale2.get()},{scale3.get()},{val}, {scale5.get()}"))
scale4.pack()

scale5 = Scale(root,
               variable=DoubleVar(value=0),
               width=10,
               length=400,
               orient=HORIZONTAL,
               from_=-90,
               to=90,
               command=lambda val: sup(f"{scale1.get()},{scale2.get()},{scale3.get()},{scale4.get()},{val}"))
scale5.pack()

root.mainloop()

# Close the serial port after the window is closed
print("Closing down")
s.close()