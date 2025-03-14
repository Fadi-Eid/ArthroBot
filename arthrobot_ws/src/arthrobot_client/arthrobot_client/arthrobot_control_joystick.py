from threading import Thread
import rclpy
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient
from pyPS4Controller.controller import Controller

class JoystickController(Controller):
    def __init__(self, node, **kwargs):
        Controller.__init__(self, **kwargs)
        self.node = node
        self.running = False
        self.servo_controller = True

        self.command_type = None
        self.command_value = 0.0

        self.start_motion()

    # Switch between controllers 
    def on_x_press(self):
        self.switch_controller_callback()

    def on_R3_up(self, val):
        self.command_type = 3
        self.command_value = -((val - 260.0) / (32760.0 - 260.0) + 0.02)
        self.start_motion()

    def on_R3_down(self, val):
        self.command_type = 3
        self.command_value = -((val - 260.0) / (32760.0 - 260.0) - 0.02)
        self.start_motion()

    def on_R3_right(self, val):
        self.command_type = 2
        self.command_value = (val - 260.0) / (32760.0 - 260.0) - 0.02
        self.start_motion()

    def on_R3_left(self, val):
        self.command_type = 2
        self.command_value = (val - 260.0) / (32760.0 - 260.0) + 0.02
        self.start_motion()

    def on_L3_up(self, val):
        self.command_type = 1
        self.command_value = -((val - 260.0) / (32760.0 - 260.0) + 0.02)
        self.start_motion()

    def on_L3_down(self, val):
        self.command_type = 1
        self.command_value = -((val - 260.0) / (32760.0 - 260.0) - 0.02)
        self.start_motion()

    def on_R1_press(self):
        self.command_type = 4
        self.command_value = -1.0
        self.start_motion()
    def on_R1_release(self):
        self.command_type = 4
        self.command_value = 0.0
        self.start_motion()
    def on_L1_press(self):
        self.command_type = 4
        self.command_value = 1.0
        self.start_motion()
    def on_L1_release(self):
        self.command_type = 4
        self.command_value = 0.0
        self.start_motion()


    def switch_controller_callback(self):
        print("switching controllers")
        if self.servo_controller:
            self.node.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        else:
            self.node.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
        self.servo_controller = not self.servo_controller


    def start_motion(self):
        self.running = True
        Thread(target=self.send_commands, daemon=True).start()

    def send_commands(self):
        while self.running:
            if abs(self.command_value) > 0.1:
                self.node.setCartesianGoal(self.command_type, self.command_value)
            self.node.get_clock().sleep_for(rclpy.time.Duration(seconds=0.15))

    def stop_motion(self):
        self.command_value = 0.0
        self.running = False


def main():

    rclpy.init()
    node = ArthrobotServoClient()
      # Start ROS 2 spinning in a separate thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    controller = JoystickController(node, interface="/dev/input/js1", connecting_using_ds4drv=False)
    # you can start listening before controller is paired, as long as you pair it within the timeout window
    controller.listen(timeout=1)

    # Cleanup after GUI closes
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

