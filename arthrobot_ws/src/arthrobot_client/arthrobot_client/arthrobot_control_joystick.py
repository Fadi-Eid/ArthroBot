from threading import Thread
import rclpy
import time
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient
from pyPS4Controller.controller import Controller

class JoystickController(Controller):
    def __init__(self, node, **kwargs):
        Controller.__init__(self, **kwargs)
        self.node = node
        self.servo_controller = True
        self.gripper_open = False

        self.cartesian_cmd_type = None
        self.cartesian_cmd_value = 0.0

        self.joint_cmd_type = None
        self.joint_cmd_value = 0.0

        self.running = True
        self.control_thread = Thread(target=self.send_commands_loop, daemon=True)
        self.control_thread.start()

    def send_commands_loop(self):
        while self.running:
            if abs(self.cartesian_cmd_value) > 0.06:
                self.node.setCartesianGoal(self.cartesian_cmd_type, self.cartesian_cmd_value)
            if abs(self.joint_cmd_value) > 0.06:
                self.node.setJointGoal(self.joint_cmd_type, self.joint_cmd_value)
            time.sleep(0.1)

    # Switch between controllers (position if servoing and trajectory if other)
    def on_x_press(self):
        self.switch_controller_callback()
    
    # call task to get outside singularity in case arthrobot is stuck
    def on_up_arrow_press(self):
        if self.servo_controller is True:
            self.switch_controller_callback()
        self.node.execute_task(3)
        time.sleep(3.5)
        self.switch_controller_callback()

    # Open/Close the gripper
    def on_square_press(self):
        if self.servo_controller is True:
            self.switch_controller_callback()
        if self.gripper_open is True:
            self.node.execute_task(2)
            self.gripper_open = False
            time.sleep(0.3) # TODO: I think this is not best practice
            self.switch_controller_callback()
        else:
            self.node.execute_task(1)
            self.gripper_open = True
            time.sleep(0.3)
            self.switch_controller_callback()
    
    # Move along the x-axis
    def on_L3_up(self, val):
        self.cartesian_cmd_type = 1
        self.cartesian_cmd_value = -((val - 260.0) / (32760.0 - 260.0) + 0.02)
    def on_L3_down(self, val):
        self.cartesian_cmd_type = 1
        self.cartesian_cmd_value = -((val - 260.0) / (32760.0 - 260.0) - 0.02)
    
    # Move along the z-axis
    def on_R3_up(self, val):
        self.cartesian_cmd_type = 3
        self.cartesian_cmd_value = -((val - 260.0) / (32760.0 - 260.0) + 0.02)
    def on_R3_down(self, val):
        self.cartesian_cmd_type = 3
        self.cartesian_cmd_value = -((val - 260.0) / (32760.0 - 260.0) - 0.02)

    # Move along the y-axis
    def on_R3_right(self, val):
        self.cartesian_cmd_type = 2
        self.cartesian_cmd_value = (val - 260.0) / (32760.0 - 260.0) - 0.02
    def on_R3_left(self, val):
        self.cartesian_cmd_type = 2
        self.cartesian_cmd_value = (val - 260.0) / (32760.0 - 260.0) + 0.02

    # Rotate the gripper around the x-axis
    def on_R1_press(self):
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = -1.0
    def on_R1_release(self):
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 0.0
    def on_L1_press(self):
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 1.0
    def on_L1_release(self):
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 0.0

    # Rotate the waist
    def on_R2_press(self, value):
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.99
    def on_L2_press(self, value):
        self.joint_cmd_type = 1
        self.joint_cmd_value = -0.99
    def on_R2_release(self):
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.0
    def on_L2_release(self):
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.0

    def switch_controller_callback(self):
        print("switching controllers")
        if self.servo_controller:
            self.node.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        else:
            self.node.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
        self.servo_controller = not self.servo_controller

def main():

    rclpy.init()
    node = ArthrobotServoClient()
    # Start ROS 2 spinning in a separate thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    controller = JoystickController(node, interface="/dev/input/js0", connecting_using_ds4drv=False)
    # pair it within the timeout window
    controller.listen(timeout=10)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

