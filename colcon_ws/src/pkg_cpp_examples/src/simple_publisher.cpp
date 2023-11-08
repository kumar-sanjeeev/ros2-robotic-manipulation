#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimplePub: public rclcpp::Node
{
public:
    SimplePub(): Node("simple_publisher"), counter_{0}
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&SimplePub::timerCallback, this));
        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

    // class methods
    void timerCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS 2 - counter " + std::to_string(counter_ ++);
        pub_->publish(msg);
    }

private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}