import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# create the node class
class SimplePublisher(Node): # inherits from the rclpy Node class
    # define the constructor of the class
    def __init__(self):
        # call the constructor of the node class
        super().__init__("simple_publisher")    # name of the node
        # create a publisher
        self.pub_ = self.create_publisher(String, "chatter", 10)    # type of th message
                                                                  # and the name of the topic to publish to 
        self.counter_ = 0
        self.frequency_ = 1.0   # publishing frequency in Hz
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        # timer object
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
    
    # define the callback function to be executed at the specified frequency
    def timerCallback(self):
        msg = String()
        msg.data = "Hello from ROS2 - counter %d" % self.counter_
        # make the publisher object publish the message
        self.pub_.publish(msg)
        self.counter_ += 1  # increment the counter

# define the main function
def main():
    # initialize ROS from RCLPY
    rclpy.init()    # instantiate the communication with ROS
    # instantiate an object from the SimplePublisher class
    simple_publisher = SimplePublisher()
    # keep the node up and running
    rclpy.spin(simple_publisher)
    # destroy the publisher node when the execution is terminated
    simple_publisher.destroy_node()
    # shutdown ROS 2
    rclpy.shutdown()


# call the main function
if __name__ == "__main__":
    main()
