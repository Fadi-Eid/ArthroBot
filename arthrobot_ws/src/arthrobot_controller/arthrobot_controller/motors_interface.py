#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arthrobot_interfaces.msg import ArthrobotPositionCommand
import serial

# To allow usage of the USB hadrware without sudo, check the link below
# https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0


serial_port = "/dev/ttyUSB0"
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
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("Motors Interface Node Started")

    def timer_callback(self):
        if not self.isSim:
            self.receive_data()
        else:
            return

    def receive_data(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode('utf-8').strip()
                parts = data.split(',') 
                try:
                    # Attempt to convert the split string parts into float values
                    msg = ArthrobotPositionCommand()
                    msg.joint1_pos = float(parts[0])
                    msg.joint2_pos = float(parts[1])
                    msg.joint3_pos = float(parts[2])
                    msg.joint4_pos = float(parts[3])
                    msg.joint5_pos = float(parts[4])
                    self.motor_feedback.publish(msg) # real feedback
                except (ValueError, IndexError):
                    self.get_logger().info("\033[31mCould not get data. Maybe data rate is too high. Try to lower it\033[0m")
                    pass
        except:
            self.get_logger().info("\033[31mHardware undetectable, switching to simulation\033[0m")
            self.get_logger().info("\033[1;33mRunning in simulation\033[0m")
            self.isSim = True


    def motor_commands_callback(self, msg):
        joint1_cmd = msg.joint1_pos
        joint2_cmd = msg.joint2_pos
        joint3_cmd = msg.joint3_pos
        joint4_cmd = msg.joint4_pos
        joint5_cmd = msg.joint5_pos

        if self.isSim:
            self.motor_feedback.publish(msg) # fake feedback
        else:
            self.send_data(self.ser, joint1_cmd, joint2_cmd, joint3_cmd, joint4_cmd, joint5_cmd)
    
    # Function to send data in the format float1,float2,float3
    def send_data(self, f1, f2, f3): # to the motor
        data = "{:.2f},{:.2f},{:.2f}\n".format(f1, f2, f3)
        try:
            self.ser.write(data.encode('utf-8'))  # Send the data as bytes
        except:
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
