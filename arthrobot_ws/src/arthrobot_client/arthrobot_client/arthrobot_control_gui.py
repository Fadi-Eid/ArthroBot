import tkinter as tk
from threading import Thread
import rclpy
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient

class ArthrobotControlGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = False
        self.current_command = None

        root.title("Arthrobot Control GUI")

        # Cartesian Control Buttons
        self.cartesian_buttons = {
            "X+": (1, 0, lambda: self.start_cartesian(1)),
            "X-": (1, 1, lambda: self.start_cartesian(2)),
            "Z+": (0, 2, lambda: self.start_cartesian(3)),
            "Z-": (2, 2, lambda: self.start_cartesian(4)),
            "Rot+Y": (0, 1, lambda: self.start_cartesian(5)),
            "Rot-Y": (2, 1, lambda: self.start_cartesian(6)),
            "Y+": (0, 0, lambda: self.start_cartesian(7)),
            "Y-": (2, 0, lambda: self.start_cartesian(8))
        }

        for text, (row, col, cmd) in self.cartesian_buttons.items():
            btn = tk.Button(root, text=text, width=10)
            btn.grid(row=row, column=col, padx=5, pady=5)
            btn.bind('<ButtonPress-1>', lambda e, c=cmd: c())
            btn.bind('<ButtonRelease-1>', lambda e: self.stop_motion())

        # Joint Control Buttons
        self.joint_buttons = {
            "Joint 1": (3, 0, lambda: self.start_joint(1)),
            "Joint 2": (3, 1, lambda: self.start_joint(2)),
            "Joint 3": (3, 2, lambda: self.start_joint(3)),
            "Joint 4": (4, 0, lambda: self.start_joint(4)),
            "Joint 5": (4, 1, lambda: self.start_joint(5))
        }

        for text, (row, col, cmd) in self.joint_buttons.items():
            btn = tk.Button(root, text=text, width=10)
            btn.grid(row=row, column=col, padx=5, pady=5)
            btn.bind('<ButtonPress-1>', lambda e, c=cmd: c())
            btn.bind('<ButtonRelease-1>', lambda e: self.stop_motion())

        # Reverse Direction Button
        self.reverse_btn = tk.Button(root, text="Reverse Direction", command=self.node.reverse_direction, width=15)
        self.reverse_btn.grid(row=5, column=1, padx=5, pady=10)

    def start_cartesian(self, command):
        self.running = True
        self.current_command = command
        Thread(target=self.send_cartesian_command, daemon=True).start()
    
    def start_joint(self, joint):
        self.running = True
        self.current_command = joint
        Thread(target=self.send_joint_command, daemon=True).start()

    def send_cartesian_command(self):
        while self.running:
            self.node.setCartesianGoal(self.current_command)
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.04))

    def send_joint_command(self):
        while self.running:
            self.node.setJointGoal(self.current_command, self.node.direction)
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.04))

    def stop_motion(self):
        self.running = False

def main():

    rclpy.init()
    node = ArthrobotServoClient()
    root = tk.Tk()
    app = ArthrobotControlGUI(root, node)
    root.mainloop()
    node.destroy_node()
    node.shutdown_hook()

if __name__ == "__main__":
    main()
