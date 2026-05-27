/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/

#include "sealien_mavlink_joy_bridge/mixed_io_joy_bridge_core.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MixedIoJoyBridge>("mixed_io_joy_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
