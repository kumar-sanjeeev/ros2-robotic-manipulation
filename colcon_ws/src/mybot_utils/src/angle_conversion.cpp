#include <rclcpp/rclcpp.hpp>
#include "mybot_msgs/srv/euler_to_quaternion.hpp"
#include "mybot_msgs/srv/quaternion_to_euler.hpp"
#include <memory>
#include <tf2/utils.h>

using namespace std::placeholders;

class AngleConvertor: public rclcpp::Node
{
public:
    AngleConvertor(): Node("angles_conversion_service")
    {
        euler_to_quaterion_ = this->create_service<mybot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AngleConvertor::euler_to_quaternion_callback, this, _1, _2));
        quaternion_to_euler_ = this->create_service<mybot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AngleConvertor::quaternion_to_euler_callback, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle Conversion service is ready to avail...");

    }
private:
    // declaring the service object
    rclcpp::Service<mybot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaterion_;
    rclcpp::Service<mybot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    // declaring the callback function for euler to quaternion service server
    void euler_to_quaternion_callback(const std::shared_ptr<mybot_msgs::srv::EulerToQuaternion::Request> req,
                                      const std::shared_ptr<mybot_msgs::srv::EulerToQuaternion::Response> resp)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert the euler angles roll: "<< req->roll << " pitch:" << req->pitch << " yaw:" << req->yaw << " into quaternions");
        tf2::Quaternion q;
        q.setRPY(req->roll, req->pitch, req->yaw);
        resp->x = q.getX();
        resp->y = q.getY();
        resp->z = q.getZ();
        resp->w = q.getW();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding quaternions are x: "<< resp->x << " y:" << resp->y << " z:" << resp->z << " w:" << resp->w );
    }

    // declaring the callback function for quaternion to euler service server
    void quaternion_to_euler_callback(const std::shared_ptr<mybot_msgs::srv::QuaternionToEuler::Request> req,
                                      const std::shared_ptr<mybot_msgs::srv::QuaternionToEuler::Response> resp)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Request to convert the quaternions x: "<< req->x << " y:" << req->y << " z:" << req->z << " w:" << req->w << " into eulers");
        tf2::Quaternion q(req->x, req->y, req->z, req->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(resp->roll, resp->pitch, resp->yaw);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Corresponding eulers are roll: "<< resp->roll << " pitch:" << resp->pitch << " yaw:" << resp->yaw);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto angle_convertor = std::make_shared<AngleConvertor>();
    rclcpp::spin(angle_convertor);
    rclcpp::shutdown();
    return 0;

}
