/***** (C) Copyright, Sealien Robotics(Guangzhou) Co.,Ltd. ******source file****/
/* adc_to_joy_mapper 可执行入口 */

#include "sealien_mavlink_joy_bridge/adc_to_joy_mapper.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdcToJoyMapper>("adc_to_joy_mapper");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
