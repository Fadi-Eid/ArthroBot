from threading import Thread
import rclpy
import time
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient
from pyPS4Controller.controller import Controller
from enum import Enum

class ControllerType(Enum):
    TRAJECTORY = "arthrobot_controller"
    POSITION = "arthrobot_servo_controller"

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

        self.command_threshold = 0.05

        self.running = True
        self.control_thread = Thread(target=self.send_commands_loop, daemon=True)
        self.control_thread.start()

    def send_commands_loop(self):
        while self.running:
            if abs(self.cartesian_cmd_value) > self.command_threshold:
                self.node.setCartesianGoal(self.cartesian_cmd_type, self.cartesian_cmd_value)
            if abs(self.joint_cmd_value) > self.command_threshold:
                self.node.setJointGoal(self.joint_cmd_type, self.joint_cmd_value)
            time.sleep(0.1)


    # Switch between controllers position follower and trajectory controllers
    def on_x_press(self):
        if self.servo_controller == True:
            self.switch_controller_callback(ControllerType.TRAJECTORY)
        else:
            self.switch_controller_callback(ControllerType.POSITION)
    
    # call task to get outside singularity in case arthrobot is stuck
    def on_up_arrow_press(self):
        self.switch_controller_callback(ControllerType.TRAJECTORY)
        self.node.execute_task(3)

    # Open/Close the gripper
    def on_square_press(self):
        self.switch_controller_callback(ControllerType.TRAJECTORY)
        if self.gripper_open == True:
            self.node.execute_task(2)
            self.gripper_open = False
        else:
            self.node.execute_task(1)
            self.gripper_open = True

    def move_cartesian(self, axis: int, val: float, offset: float = 0.02, invert: bool = False):
        self.switch_controller_callback(ControllerType.POSITION)
        self.cartesian_cmd_type = axis
        sign = -1 if invert else 1
        self.cartesian_cmd_value = sign * ((val - 260.0) / (32760.0 - 260.0) + offset)

    # Move along the x-axis
    def on_L3_up(self, val): self.move_cartesian(1, val, invert=True)
    def on_L3_down(self, val): self.move_cartesian(1, val, offset=-0.02, invert=True)

    # Move along the z-axis
    def on_R3_up(self, val): self.move_cartesian(3, val, invert=True)
    def on_R3_down(self, val): self.move_cartesian(3, val, offset=-0.02, invert=True)

    # Move along the y-axis
    def on_R3_right(self, val): self.move_cartesian(2, val, offset=-0.02)
    def on_R3_left(self, val): self.move_cartesian(2, val, offset=0.02)

    # Rotate the end effector around the x-axis
    def on_R1_press(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = -1.0
    def on_R1_release(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 0.0
    def on_L1_press(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 1.0
    def on_L1_release(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.cartesian_cmd_type = 4
        self.cartesian_cmd_value = 0.0

    # Rotate the waist
    def on_R2_press(self, value):
        self.switch_controller_callback(ControllerType.POSITION)
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.99
    def on_L2_press(self, value):
        self.switch_controller_callback(ControllerType.POSITION)
        self.joint_cmd_type = 1
        self.joint_cmd_value = -0.99
    def on_R2_release(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.0
    def on_L2_release(self):
        self.switch_controller_callback(ControllerType.POSITION)
        self.joint_cmd_type = 1
        self.joint_cmd_value = 0.0

    def switch_controller_callback(self, controllerType : ControllerType):
        if controllerType == ControllerType.POSITION and self.servo_controller == False:
            self.node.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
            print("Switching to trajectory controller")
            self.servo_controller = True
        elif controllerType == ControllerType.TRAJECTORY and self.servo_controller == True:
            self.node.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
            print("switching to position controller")
            self.servo_controller = False

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
    controller.switch_controller_callback(ControllerType.TRAJECTORY)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

