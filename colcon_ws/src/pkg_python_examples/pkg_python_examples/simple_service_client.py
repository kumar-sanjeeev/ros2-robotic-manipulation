import rclpy
from rclpy.node import Node
from mybot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    """
    This class demonstrates how the service client can be implemented
    in ROS2 framework
    """
    def __init__(self, a, b):
        # this is class constructor
        super().__init__("simple_service_client")
        self.service_client_ = self.create_client(AddTwoInts, "add_two_ints")

        # verify check: if server is available to provide the service
        while not self.service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service not avilable yet.. waiting more")
        
        # now service is available to client
        self.req_ = AddTwoInts.Request()
        self.req_.a = int(a)
        self.req_.b = int(b)

        # restoring the result of call_async function in the future_ variable
        self.future_ = self.service_client_.call_async(self.req_)
        # add the callback function
        self.future_.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """
        This function will execute once we get the repose from service 
        server
        """
        self.get_logger().info(f"Service Response: {future.result().sum}")

def main():
    rclpy.init()
    if len(sys.argv) != 3:
        print("Wrong number of arguments! Usage: simple_service_client A B")
        return -1
    simple_service_client = SimpleServiceClient(sys.argv[1], sys.argv[2])
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()



