#include "avt_vimba_camera/image_subscriber_node.hpp"
#include <sstream>


namespace avt_vimba_camera
{

ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("image_subscriber", options)
{
  // Create QoS profile matching the publisher
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.reliable();

  rclcpp::SubscriptionOptions sub_options;
  sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  // Create the image subscription
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image/ptr",
    qos,
    std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1),
    sub_options
  );

  // Create the demo subscription
  subscription_demo_ = this->create_subscription<std_msgs::msg::Int32>(
    "image/demo",
    qos,
    std::bind(&ImageSubscriberNode::demoCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Image subscriber node started with IPC enabled");
}

void ImageSubscriberNode::imageCallback(sensor_msgs::msg::Image::UniquePtr msg)
{
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null image message");
        return;
    }

    std::stringstream ss;
    ss << "0x" << std::hex << reinterpret_cast<std::uintptr_t>(msg.get());
    
    // Store values locally before logging
    const auto width = msg->width;
    const auto height = msg->height;
    const auto encoding = msg->encoding;
    const auto addr = ss.str();
    
    RCLCPP_INFO(this->get_logger(), 
        "Received image: %dx%d, encoding: %s, address: %s",
        width, height, encoding.c_str(), addr.c_str());
}

void ImageSubscriberNode::demoCallback(std_msgs::msg::Int32::UniquePtr msg)
{
  std::stringstream ss;
  ss << "0x" << std::hex << reinterpret_cast<std::uintptr_t>(msg.get());
  
  RCLCPP_INFO(this->get_logger(), 
    "Received demo message: %d, address: %s",
    msg->data, ss.str().c_str());
}

}  // namespace avt_vimba_camera


// Add these lines at the end of the file
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(avt_vimba_camera::ImageSubscriberNode)
