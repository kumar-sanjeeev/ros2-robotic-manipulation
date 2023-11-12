import rclpy
from rclpy.node import Node
from mybot_msgs.srv import AddTwoInts

class ServiceAddNo(Node):
    """
    This class demonstrates  an example to showcase how service 
    can be used to add the two number and return the response to
    client.
    """
    def __init__(self):
        # this is class constructor
        super().__init__(node_name="service_add_number")
        self.service_ = self.create_service(AddTwoInts, srv_name="add_two_ints", callback=self.service_callback)
        self.get_logger().info(f"Service add_two_ints is ready")
    
    def service_callback(self, req, resp):
        """
        This is service callback function, executed when client request for the service
        It receives the request from the client and returns the reponse back to the client.
        """
        self.get_logger().info(f"New Message received from the client a: {req.a}, and b: {req.b}")
        resp.sum = req.a + req.b
        self.get_logger().info(f"Returning sum: {resp.sum}")
        return resp

def main():
    rclpy.init()
    service = ServiceAddNo()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

