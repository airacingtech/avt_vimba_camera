#include <rclcpp/rclcpp.hpp>
#include "avt_vimba_camera/image_subscriber_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<avt_vimba_camera::ImageSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}