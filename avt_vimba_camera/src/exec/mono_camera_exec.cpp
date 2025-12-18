#include "avt_vimba_camera/mono_camera_node.hpp"

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<avt_vimba_camera::MonoCameraNode>(options);
  rclcpp::spin(node);

  return 0;
}
