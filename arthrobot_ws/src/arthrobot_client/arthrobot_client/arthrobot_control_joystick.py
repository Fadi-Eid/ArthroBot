from threading import Thread
import rclpy
from arthrobot_client.arthrobot_servo_client import ArthrobotServoClient
from pyPS4Controller.controller import Controller

class JoystickController(Controller):
    def __init__(self, node, **kwargs):
        Controller.__init__(self, **kwargs)
        self.node = node
        self.running = False
        self.current_command = None
        self.servo_controller = True
        self.joint_directions = {1: 0, 2: 0, 3: 0, 4: 0, 5: 0}

    def on_x_press(self):
        self.switch_controller_callback()

    def on_R3_up(self, val):
        if val < -22000:
            self.start_cartesian(3)
        else:
            self.stop_motion()

    def on_R3_down(self, val):
        if val > 22000:
            self.start_cartesian(4)
        else:
            self.stop_motion()

    def on_R3_right(self, val):
        if val > 22000:
            self.start_cartesian(7)
        else:
            self.stop_motion()

    def on_R3_left(self, val):
        if val < -22000:
            self.start_cartesian(8)
        else:
            self.stop_motion()

    def switch_controller_callback(self):
        print("switching controllers")
        if self.servo_controller:
            self.node.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        else:
            self.node.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
        self.servo_controller = not self.servo_controller
        
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
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.02))


    def start_cartesian(self, command):
        self.running = True
        self.current_command = command
        Thread(target=self.send_cartesian_command, daemon=True).start()

    def send_cartesian_command(self):
        while self.running:
            self.node.setCartesianGoal(self.current_command)
            self.node.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.02))

    def stop_motion(self):
        self.running = False


def main():

    rclpy.init()
    node = ArthrobotServoClient()
      # Start ROS 2 spinning in a separate thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    controller = JoystickController(node, interface="/dev/input/js1", connecting_using_ds4drv=False)
    # you can start listening before controller is paired, as long as you pair it within the timeout window
    controller.listen(timeout=60)

    # Cleanup after GUI closes
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

