#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arthrobot_interfaces.msg import ArthrobotPositionCommand
import serial

# To allow usage of the USB hadrware without sudo, check the link below
# https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0


serial_port = "/dev/ttyACM0"
baud_rate = 115200

class MotorsInterface(Node):

    def __init__(self):
        super().__init__("motors_interface_node")

        self.isSim = None # Simulation / Hardware flag (Use launch arguments later)

        try:
            self.ser = serial.Serial(serial_port, baud_rate)
            self.isSim = False
            self.get_logger().info("\033[1;33mRunning on real hardware\033[0m")
        except:
            self.isSim = True
            self.get_logger().info("\033[1;33mRunning in simulation\033[0m")

        self.motor_commands = self.create_subscription(ArthrobotPositionCommand, "arthrobot_hardware_commands", self.motor_commands_callback, 10)
        self.motor_feedback = self.create_publisher(ArthrobotPositionCommand, "arthrobot_hardware_feedback", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Motors Interface Node Started")

    def timer_callback(self):
        if self.isSim:
            try:
                self.ser = serial.Serial(serial_port, baud_rate)
                self.isSim = False
                self.get_logger().info("\033[1;33mRunning on real hardware\033[0m")
            except:
                pass

    def motor_commands_callback(self, msg):
        joint1_cmd = msg.waist_pos * (57.2957) + 90
        joint2_cmd = msg.shoulder_pos * (57.2957) + 90
        joint3_cmd = -msg.forearm_pos * (57.2957) + 90
        joint4_cmd = msg.wrist_pos * (57.2957) + 90
        joint5_cmd = msg.palm_pos * (57.2957) + 90
        joint6_cmd = 180 - msg.gripper_pos * (57.2957)

        if not self.isSim:
             self.send_data(joint1_cmd, joint2_cmd, joint3_cmd, joint4_cmd, joint5_cmd, joint6_cmd)
        
        self.motor_feedback.publish(msg) # fake feedback because Arthrobot's motors doesn't provide feedback
           
    
    # Function to send data in the format float1,float2,float3
    def send_data(self, f1, f2, f3, f4, f5, f6): # to the motor
        data = "{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(f1, f2, f3, f4, f5, f6)
        # self.get_logger().info(f"data: {data}")
        try:
            self.ser.write(data.encode())  # Send the data as bytes
        except:
            pass
            self.get_logger().info("\033[31mHardware undetectable, switching to simulation\033[0m")
            self.isSim = True
            self.get_logger().info("\033[1;33mRunning in simulation\033[0m")



def main():
    rclpy.init()

    motor_interface_node = MotorsInterface()
    rclpy.spin(motor_interface_node)
    
    motor_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
