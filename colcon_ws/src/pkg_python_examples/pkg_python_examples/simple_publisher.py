import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    This class implements the example to show how publisher works
    in the ros2.
    """
    def __init__(self):
        # this is class constructor
        super().__init__(node_name="simple_publisher")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.frequency_ = 1.0
        self.get_logger().info(f"Publishing at rate of {self.frequency_}")

        # create the timer object
        self.timer_ = self.create_timer(timer_period_sec=self.frequency_, callback=self.timer_callback)
    
    def timer_callback(self):
        msg = String()
        msg.data = f"Hello ROS2, counter: {self.counter_}"
        self.publisher_.publish(msg)
        self.counter_ += 1

def main():
    # start the ros communication
    rclpy.init()
    # create an instance of the class
    simple_pub = SimplePublisher()
    # spin the node to run continuously
    rclpy.spin(simple_pub)
    # destroy the node
    simple_pub.destroy_publisher()
    # shutdown the ros communication
    rclpy.shutdown()

if __name__ == "__main__":
    main()

