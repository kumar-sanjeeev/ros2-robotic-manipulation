import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class Param(Node):
    """
    This class implements the example to show how we can use the ROS
    parameters to configure some ROS Node
    """
    def __init__(self):
        # this is class constructor
        super().__init__(node_name="simple_python_parameters")
        self.declare_parameter(name="age_param", value=27)
        self.declare_parameter(name="name_param", value="sanjeev")
        
        self.add_on_set_parameters_callback(callback=self.param_change_callback)

    def param_change_callback(self, params):
        """
        This callback function will execute whenever the
        node parameters get changed.

        It will also return the result: in form of changed value of params.
        """
        result = SetParametersResult()

        for param in params:
            if param.name == "age_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Param age_param changed! New value is: {param.value}")
                result.successful = True
            if param.name == "name_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Param name_param changed! New value is: {param.value}")
                result.successful = True
        
        return result

def main():
    rclpy.init()
    simple_param = Param()
    rclpy.spin(simple_param)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            
