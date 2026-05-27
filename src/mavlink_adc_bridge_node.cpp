#include "sealien_mavlink_joy_bridge/mavlink_adc_bridge_core.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkAdcBridge>("mavlink_adc_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
