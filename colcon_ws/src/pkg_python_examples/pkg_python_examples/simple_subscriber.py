import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSub(Node):
    """
    This class implements the example to show how the subscriber
    works in ROS2
    """
    def __init__(self):
        # this is class constuctor
        super().__init__(node_name="simple_subscriber")
        self.sub_ = self.create_subscription(String, "chatter", self.sub_callback, 10)
    
    def sub_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main():
    rclpy.init()
    simple_sub = SimpleSub()
    rclpy.spin(simple_sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
