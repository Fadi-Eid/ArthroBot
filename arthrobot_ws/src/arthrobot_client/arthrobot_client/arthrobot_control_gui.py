import tkinter as tk
from tkinter import ttk
from threading import Thread
import rclpy
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient

class ArthrobotControlGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = False
        self.current_command = None
        self.servo_controller = True

        root.title("Arthrobot Control GUI")
        root.geometry("600x400")  # Optional: set default size
        root.minsize(500, 400)    # Optional: set minimum window size

        # Configure grid weight for responsiveness
        for i in range(4):
            root.columnconfigure(i, weight=1)
        for i in range(4):
            root.rowconfigure(i, weight=1)

        self.joint_directions = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

        # --- Joint Control Sliders ---
        joint_frame = ttk.LabelFrame(root, text="Joint Control")
        joint_frame.grid(row=0, column=0, columnspan=4, padx=10, pady=10, sticky="ew")

        self.sliders = {}
        for i in range(1, 6):  # Joint 1 to 5
            ttk.Label(joint_frame, text=f"Joint {i}:").grid(row=i-1, column=0, padx=5, pady=5, sticky="w")
            slider = ttk.Scale(joint_frame, from_=-1.0, to=1.0, orient="horizontal", length=300,
                               command=lambda val, j=i: self.update_joint_goal(j, float(val)))
            slider.grid(row=i-1, column=1, padx=5, pady=5, sticky="ew")
            slider.set(0)  # Default to 0
            self.sliders[i] = slider

        # Make sliders responsive
        joint_frame.columnconfigure(1, weight=1)

        # --- Cartesian Control Buttons ---
        cartesian_frame = ttk.LabelFrame(root, text="Cartesian Control")
        cartesian_frame.grid(row=1, column=0, columnspan=4, padx=10, pady=10, sticky="ew")

        self.cartesian_buttons = {
            "X+":    (0, 0, lambda: self.start_cartesian(1)),
            "X-":    (1, 0, lambda: self.start_cartesian(2)),
            "Z+":    (0, 2, lambda: self.start_cartesian(3)),
            "Z-":    (1, 2, lambda: self.start_cartesian(4)),
            "Y+":    (0, 1, lambda: self.start_cartesian(7)),
            "Y-":    (1, 1, lambda: self.start_cartesian(8)),
            "Rot+":  (0, 3, lambda: self.start_cartesian(5)),
            "Rot-":  (1, 3, lambda: self.start_cartesian(6)),
        }

        for text, (row, col, cmd) in self.cartesian_buttons.items():
            btn = ttk.Button(cartesian_frame, text=text)
            btn.grid(row=row, column=col, padx=5, pady=5, sticky="ew")
            btn.bind('<ButtonPress-1>', lambda e, c=cmd: c())
            btn.bind('<ButtonRelease-1>', lambda e: self.stop_motion())

<<<<<<< HEAD
        # responsive
=======
        # Make buttons responsive
>>>>>>> origin/main
        for i in range(4):
            cartesian_frame.columnconfigure(i, weight=1)

        # --- Optional: Gripper Slider (Vertical) ---
        # gripper_frame = ttk.LabelFrame(root, text="Gripper Control")
        # gripper_frame.grid(row=0, column=4, rowspan=2, padx=10, pady=10, sticky="ns")

        # ttk.Label(gripper_frame, text="Gripper:").pack(pady=5)
        # self.vertical_slider = ttk.Scale(gripper_frame, from_=0, to=100, orient="vertical", length=200,
        #                                  command=self.vertical_slider_callback)
        # self.vertical_slider.pack(padx=10, pady=10)
        # self.vertical_slider.set(0)

<<<<<<< HEAD
        # Swich controllers
        switch_frame = tk.Frame(root)
        switch_frame.grid(row=3, column=0, columnspan=4, pady=15)  # Separate row, centered horizontally

        switch_btn = tk.Button(
            switch_frame,
            text="Switch Controller",
            bg="black",
            fg="white",
            activebackground="gray20",
            activeforeground="white",
            width=20,
            height=1,
            command=self.switch_controller_callback
        )
        switch_btn.pack(pady=5)



############################## Callback functions ###########################################################
    def switch_controller_callback(self):
        print("switching controllers")
        if self.servo_controller:
            self.node.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        else:
            self.node.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
        self.servo_controller = not self.servo_controller
        
=======
############################## Callback functions ###########################################################

>>>>>>> origin/main
    def update_joint_goal(self, joint, velocity):
        """Callback to update the joint velocity from the slider."""
        self.joint_directions[joint] = velocity
        if velocity != 0:  # If not zero, start the motion thread
            Thread(target=self.send_joint_command, daemon=True).start()

    def send_joint_command(self):
        """Continuously publish joint velocities while sliders are adjusted."""
        while any(self.joint_directions.values()):  # run while any slder is active
            for joint, velocity in self.joint_directions.items():
                if velocity != 0:
                    self.node.setJointGoal(joint, velocity)
<<<<<<< HEAD
            # self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.001))
=======
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
>>>>>>> origin/main


    def start_cartesian(self, command):
        self.running = True
        self.current_command = command
        Thread(target=self.send_cartesian_command, daemon=True).start()

    def send_cartesian_command(self):
        while self.running:
            self.node.setCartesianGoal(self.current_command)
<<<<<<< HEAD
            # self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.001))
=======
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
>>>>>>> origin/main

    def stop_motion(self):
        self.running = False


def main():

    rclpy.init()
    node = ArthrobotServoClient()
      # Start ROS 2 spinning in a separate thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Initialize and run the Tkinter GUI
    root = tk.Tk()
    app = ArthrobotControlGUI(root, node)
    root.mainloop()

    # Cleanup after GUI closes
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

