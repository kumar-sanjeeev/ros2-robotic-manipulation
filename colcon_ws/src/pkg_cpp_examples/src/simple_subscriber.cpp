#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class SimpleSub: public rclcpp::Node
{
public:
    SimpleSub() : Node("simple_subscriber")
    {
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSub::msgCallback, this, _1));
    }

    void msgCallback(const std_msgs::msg::String &msg) const
    {
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " <<  msg.data.c_str());
    }

    
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}