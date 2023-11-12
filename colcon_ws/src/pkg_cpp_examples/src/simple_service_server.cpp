#include <rclcpp/rclcpp.hpp>
#include "mybot_msgs/srv/add_two_ints.hpp"
#include <memory> // for using the SharedPtr

using namespace std::placeholders ;

class SimpleServiceServer: public rclcpp::Node
{
public:
    SimpleServiceServer(): Node("simple_service_server")
    {
        service_ = this->create_service<mybot_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::service_callback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service add_two_ints is Ready");
    }

private:
    rclcpp::Service<mybot_msgs::srv::AddTwoInts>::SharedPtr service_;

    void service_callback(const std::shared_ptr<mybot_msgs::srv::AddTwoInts::Request> req,
                          const std::shared_ptr<mybot_msgs::srv::AddTwoInts::Response> resp)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "New Request received a: "<< req->a << " b:"<< req->b);
        resp->sum = req->a + req->b;
        RCLCPP_INFO_STREAM(this->get_logger(), "Returning sum: "<< resp->sum);
    }

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}