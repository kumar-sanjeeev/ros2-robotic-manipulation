#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>

using std::placeholders::_1;

class SimpleParameter: public rclcpp::Node
{
public:
    SimpleParameter(): Node("simple_param")
    {
        declare_parameter<int>("age_param", 27);
        declare_parameter<std::string>("name_param", "sanjeev");
        param_callback_handle_= add_on_set_parameters_callback(std::bind(&SimpleParameter::param_change_callback, this, _1));
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult param_change_callback (const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for (const auto& param: parameters)
        {
            if(param.get_name() == "age_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param age_param changed! New value is: "<< param.as_int());
                result.successful = true;
            }
            else if(param.get_name() == "name_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param name_apram changed! New value is: "<< param.as_string());
                result.successful = true;
            }
        }

        return result;
    }

};


int main(int argc, char* argv [])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}