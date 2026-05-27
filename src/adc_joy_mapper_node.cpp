#include "sealien_mavlink_joy_bridge/adc_joy_mapper_core.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdcJoyMapper>("adc_joy_mapper");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
