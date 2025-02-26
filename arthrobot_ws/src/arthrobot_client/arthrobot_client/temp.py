import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import sys, signal
import tkinter as tk
from threading import Thread

class ArthrobotServoClient(Node):
    def __init__(self):
        super().__init__("arthrobot_servo_client_node")

        signal.signal(signal.SIGINT, self.shutdown_hook)

        self.direction = 1
        self.max_service_fetch_attempts = 2

        # Create joint/cartesian command topic publishers
        self.cartesian_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)

        # Create the servo activation client
        self.servo_activate_client = self.create_client(Trigger, "/servo_node/start_servo")
        # Create the switch controller client
        self.switch_controllers_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        # Switch to arthrobot_servo_controller
        self.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
        # Activate the servo node
        self.activate_servo_node()

    def reverse_direction(self):
        self.direction *= -1
        self.get_logger().info(f"Direction switched to {self.direction}")

    def switch_controller(self, start_controller, stop_controller):
        self.get_logger().info(f"Switching to {start_controller}")
        attempts = 0
        while attempts < self.max_service_fetch_attempts:
            if self.switch_controllers_client.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Switch controllers service not available, waiting again...')
            attempts += 1
        if attempts == self.max_service_fetch_attempts:
            self.get_logger().error('Switch controllers service not available after maximum attempts. Shutting down node.')
            rclpy.shutdown()
            sys.exit(0)

        switch_controller_req = SwitchController.Request()
        switch_controller_req.start_controllers = [start_controller]
        switch_controller_req.stop_controllers = [stop_controller]
        switch_controller_req.strictness = 2
        switch_controller_future = self.switch_controllers_client.call_async(switch_controller_req)
        rclpy.spin_until_future_complete(self, switch_controller_future)

        if switch_controller_future.result() is not None:
            self.get_logger().info(f"Switched to {start_controller}")
        else:
            self.get_logger().error("Failed to switch controllers")

    def activate_servo_node(self):
        self.get_logger().info("Activating Servo Node")
        attempts = 0
        while attempts < self.max_service_fetch_attempts:
            if self.servo_activate_client.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Servo activation service not available, waiting again...')
            attempts += 1
        if attempts == self.max_service_fetch_attempts:
            self.get_logger().error('Servo activation service not available after maximum attempts. Shutting down node.')
            rclpy.shutdown()
            sys.exit(0)
            
        servo_activate_req = Trigger.Request()
        servo_activate_future = self.servo_activate_client.call_async(servo_activate_req)
        rclpy.spin_until_future_complete(self, servo_activate_future)

        if servo_activate_future.result() is not None:
            self.get_logger().info("Servo Node Activated")
        else:
            self.get_logger().error("Failed to activate Servo Node")

    def shutdown_hook(self, signum=None, frame=None):
        self.get_logger().info("Shutting down...")
        self.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        rclpy.shutdown()
        sys.exit(0)  # Exit the program

    def setJointGoal(self, joint, direction):
        msg = JointJog()
        joint_names = {1: "joint_1", 2: "joint_2", 3: "joint_3", 4: "joint_4", 5: "joint_5"}

        if joint in joint_names:
            msg.joint_names.append(joint_names[joint])
        else:
            return
        
        msg.velocities.append(1.0 * direction)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.joint_pub.publish(msg)

    def setCartesianGoal(self, command):
        msg = TwistStamped()
        
        linear_velocity = 1.0 / 2
        angular_velocity = 1.0 / 2

        if command == 1:  # move in the direction of +x
            msg.twist.linear.x = linear_velocity
        elif command == 2:  # move in the direction of -x
            msg.twist.linear.x = -linear_velocity
        elif command == 3:  # move in the direction of +z
            msg.twist.linear.z = linear_velocity
        elif command == 4:  # move in the direction of -z
            msg.twist.linear.z = -linear_velocity
        elif command == 5:  # rotate around +y
            msg.twist.angular.x = angular_velocity
        elif command == 6:  # rotate around -y
            msg.twist.angular.x = -angular_velocity
        elif command == 7:  # move in the direction of +y
            msg.twist.linear.y = linear_velocity
        elif command == 8:  # move in the direction of -y
            msg.twist.linear.y = -linear_velocity

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        self.cartesian_pub.publish(msg)

class RobotControlGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.running = False
        self.current_command = None

        root.title("Robot Control GUI")

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
    RobotControlGUI(root, node)
    root.mainloop()
    node.destroy_node()
    node.shutdown_hook()

if __name__ == "__main__":
    main()

