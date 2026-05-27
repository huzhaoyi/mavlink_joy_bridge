/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/
/* mavlink_mixed_io_receiver 可执行入口 */

#include "sealien_mavlink_joy_bridge/mavlink_mixed_io_receiver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavlinkMixedIoReceiver>("mavlink_mixed_io_receiver");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
