#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arthrobot_interfaces.msg import ArthrobotPositionCommand
import serial

# To allow usage of the USB hadrware without sudo, check the link below
# https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0

class MotorsInterface(Node):

    def __init__(self):
        super().__init__("motors_interface_node")

        #self.ser = self.setup_serial("/dev/ttyUSB0")

        self.motor_commands = self.create_subscription(ArthrobotPositionCommand, "arthrobot_hardware_commands", self.motor_commands_callback, 10)
        self.motor_feedback = self.create_publisher(ArthrobotPositionCommand, "arthrobot_hardware_feedback", 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info("Motors Interface Node Started")

    def timer_callback(self):
        pass
        #self.receive_data(self.ser)

    def receive_data(self, ser):
        pass
        #if ser.in_waiting > 0:
            # data = ser.readline().decode('utf-8').strip()
            # parts = data.split(',') 
            # try:
                # Attempt to convert the split string parts into float values
                # msg = ArthrobotPositionCommand()
                # msg.joint1_pos = float(parts[0])
                # msg.joint2_pos = float(parts[1])
                # msg.joint3_pos = float(parts[2])
                # msg.joint4_pos = float(parts[3])
                # msg.joint5_pos = float(parts[4])
                # self.motor_feedback.publish(msg)

            # except (ValueError, IndexError):
            #     self.get_logger().info("\033[31mCould not parse float. Maybe data rate is too high. Try to lower it\033[0m")
            #     pass

    def motor_commands_callback(self, msg):
        joint1_cmd = msg.joint1_pos
        joint2_cmd = msg.joint2_pos
        joint3_cmd = msg.joint3_pos
        joint4_cmd = msg.joint4_pos
        joint5_cmd = msg.joint5_pos

        self.motor_feedback.publish(msg) # fake feedback for now
        
        # self.get_logger().info(f"Received command: {joint1_cmd}/{joint2_cmd}/{joint3_cmd}/{joint4_cmd}/{joint5_cmd}")
        # self.send_data(self.ser, joint1_cmd, joint2_cmd, joint3_cmd, joint4_cmd, joint5_cmd)

    def setup_serial(self, port_name, baudrate=115200):
        ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            timeout=1  # Timeout for read operations
        )
        if ser.is_open:
            self.get_logger().info("Serial port opened successfully")
        else:
            self.get_logger().error("Could not open serial port")
        # Flush both input and output buffers
        ser.reset_input_buffer() 
        ser.reset_output_buffer()  
        return ser
    
    # Function to send data in the format float1,float2,float3
    def send_data(self, ser, f1, f2, f3): # to the motor
        data = "{:.2f},{:.2f},{:.2f}\n".format(f1, f2, f3)
        #ser.write(data.encode('utf-8'))  # Send the data as bytes


def main():
    rclpy.init()

    motor_interface_node = MotorsInterface()
    rclpy.spin(motor_interface_node)
    
    motor_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
