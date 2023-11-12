#include<rclcpp/rclcpp.hpp>
#include "mybot_msgs/srv/add_two_ints.hpp"
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;


class SimpleServiceClient: public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b): Node("simple_service_client")
    {
        client_ = this->create_client<mybot_msgs::srv::AddTwoInts>("add_two_ints");
        auto req_ = std::make_shared<mybot_msgs::srv::AddTwoInts::Request>();
        req_->a = a;
        req_->b = b;

        // verify that server is available for providing the requested service
        while(!client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available. waiting for time........");
            // check if ros is still running, if not (not ok): stop waiting for service
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interupted while waiting for the service....");
                return;
            }
        }

        // this result variable will store the reponse of the server 
        // even if server hasn't completed the service yet
        auto result = client_->async_send_request(req_, std::bind(&SimpleServiceClient::response_callback, this, _1));

        // with result variable we don't knw if service is done
        // to get this information, we will add the callback function


    }
private:
    rclcpp::Client<mybot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void response_callback(rclcpp::Client<mybot_msgs::srv::AddTwoInts>::SharedFuture future)
    {
        if(future.valid())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service Response: "<< future.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service Failure");
        }
    }
};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    if(argc !=3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong Number of Arguments! Usuage: simple_service_client A B");
        return 1;
    }
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::shutdown();
    return 0;

}