#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mybot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from  tf_transformations import quaternion_from_euler, euler_from_quaternion
 
class AngleConvertor(Node):
    """
    This class implements the service server which provide two services:
    - euler angels to quaternion conversion
    - quaternion to euler angles conversion
    """
    def __init__(self):
        # this is class constructor
        super().__init__("angle_conversion_service_server")
        self.euler_to_quaternion = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.euler_to_quaternion_callback)
        self.quaternion_to_euler = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternion_to_euler_callback)
        self.get_logger().info("Angle Conversion Service is ready to avail ")
    
    def euler_to_quaternion_callback(self, req, resp):
        self.get_logger().info(f"Requested to convert the euler angles roll: {req.roll}, pitch: {req.pitch}, yaw: {req.yaw} \
                               into quaternion")
        # use function from tf_transformation for conversion
        (resp.x, resp.y, resp.z, resp.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)

        self.get_logger().info(f"Corresponding quaternion are x: {resp.x}, y: {resp.y}, z:{resp.z}, w:{resp.w}")
        return resp



    def quaternion_to_euler_callback(self, req, resp):
        self.get_logger().info(f"Requested to convert quaternions x: {req.x}, y: {req.y}, z: {req.z}, w:{req.w} \
                               into euler angles")
        
        # use function from tf_transformation for conversion
        (resp.roll, resp.pitch, resp.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])

        self.get_logger().info(f"Corresponding euler angles are roll: {resp.roll}, pitch: {resp.pitch}, yaw:{resp.yaw}")
        return resp


def main():
    rclpy.init()
    angle_convertor = AngleConvertor()
    rclpy.spin(angle_convertor)
    angle_convertor.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
