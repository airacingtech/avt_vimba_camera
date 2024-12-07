#ifndef AVT_VIMBA_CAMERA__IMAGE_SUBSCRIBER_NODE_HPP_
#define AVT_VIMBA_CAMERA__IMAGE_SUBSCRIBER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>

namespace avt_vimba_camera
{

class ImageSubscriberNode : public rclcpp::Node
{
public:
  explicit ImageSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ImageSubscriberNode() = default;

private:
  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  
  // Demo message subscription
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_demo_;

  // Callback for image messages
  void imageCallback(sensor_msgs::msg::Image::UniquePtr msg);
  
  // Callback for demo messages
  void demoCallback(std_msgs::msg::Int32::UniquePtr msg);
};

}  // namespace avt_vimba_camera

#endif  // AVT_VIMBA_CAMERA__IMAGE_SUBSCRIBER_NODE_HPP_ 