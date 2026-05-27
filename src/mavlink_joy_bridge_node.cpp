/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****
* File name          : mavlink_joy_bridge_node.cpp
* Author             : Sealien
* Brief              : node entry
********************************************************************************/

#include "sealien_mavlink_joy_bridge/mavlink_joy_bridge_core.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkJoyBridge>("mavlink_joy_bridge");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
