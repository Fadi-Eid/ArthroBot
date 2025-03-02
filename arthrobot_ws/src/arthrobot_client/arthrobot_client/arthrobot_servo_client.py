import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import sys, signal

class ArthrobotServoClient(Node):
    def __init__(self):
        super().__init__("arthrobot_servo_client_node")
        signal.signal(signal.SIGINT, self.shutdown_hook)

        self.direction = 1
        self.max_service_fetch_attempts = 2

        self.cartesian_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)

        self.servo_activate_client = self.create_client(Trigger, "/servo_node/start_servo")
        self.switch_controllers_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        self.switch_controller("arthrobot_servo_controller", "arthrobot_controller")
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
            attempts += 1
        if attempts == self.max_service_fetch_attempts:
            self.get_logger().error('Switch controllers service unavailable. Shutting down node.')
            rclpy.shutdown()
            sys.exit(0)

        switch_controller_req = SwitchController.Request()
        switch_controller_req.start_controllers = [start_controller]
        switch_controller_req.stop_controllers = [stop_controller]
        switch_controller_req.strictness = 2
        switch_controller_future = self.switch_controllers_client.call_async(switch_controller_req)
        rclpy.spin_until_future_complete(self, switch_controller_future)

    def activate_servo_node(self):
        self.get_logger().info("Activating Servo Node")
        attempts = 0
        while attempts < self.max_service_fetch_attempts:
            if self.servo_activate_client.wait_for_service(timeout_sec=1.0):
                break
            attempts += 1
        if attempts == self.max_service_fetch_attempts:
            self.get_logger().error('Servo activation service unavailable. Shutting down node.')
            rclpy.shutdown()
            sys.exit(0)

        servo_activate_req = Trigger.Request()
        servo_activate_future = self.servo_activate_client.call_async(servo_activate_req)
        rclpy.spin_until_future_complete(self, servo_activate_future)

    def setJointGoal(self, joint, direction):
        msg = JointJog()
        joint_names = {1: "joint_1", 2: "joint_2", 3: "joint_3", 4: "joint_4", 5: "joint_5"}

        if joint in joint_names:
            msg.joint_names.append(joint_names[joint])
        else:
            return
        
        msg.velocities.append(1.0 * direction)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ee_link"
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

    def shutdown_hook(self, signum=None, frame=None):
        self.get_logger().info("Shutting down...")
        self.switch_controller("arthrobot_controller", "arthrobot_servo_controller")
        rclpy.shutdown()
        sys.exit(0)
