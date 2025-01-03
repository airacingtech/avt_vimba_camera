#include "avt_vimba_camera/image_subscriber_node.hpp"
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui.hpp>  // For imshow and waitKey


namespace avt_vimba_camera
{

ImageSubscriberNode::ImageSubscriberNode(const rclcpp::NodeOptions & options)
: Node("image_subscriber", options)
{
  // Create QoS profile matching the publisher
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

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
  // subscription_demo_ = this->create_subscription<std_msgs::msg::Int32>(
  //   "image/demo",
  //   qos,
  //   std::bind(&ImageSubscriberNode::demoCallback, this, std::placeholders::_1)
  // );

  // Create a publisher to re-publish the image on "output_image" topic
  // image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ptr/output_image", 10);
  // demo_publisher_ = this->create_publisher<std_msgs::msg::Int32>("demo/output_image", 10);



  RCLCPP_INFO(this->get_logger(), "Image subscriber node started with IPC enabled");
}

void ImageSubscriberNode::imageCallback(sensor_msgs::msg::Image::UniquePtr msg)
{
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null image message");
        return;
    }
    try {
        // Convert ROS image message to OpenCV image - note the *msg to dereference
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        
        // Use frame_id as window name, fallback to "Camera Image" if empty
        std::string window_name = msg->header.frame_id.empty() ? 
            "Camera Image" : msg->header.frame_id;
        
        // Display the image
        cv::imshow(window_name, cv_ptr->image);
        cv::waitKey(1);  // Wait 1ms to allow image to display
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::stringstream ss;
    // std::stringstream ss;
    // ss << "0x" << std::hex << reinterpret_cast<std::uintptr_t>(msg.get());
    
    // // Store values locally before logging
    // const auto width = msg->width;
    // const auto height = msg->height;
    // const auto encoding = msg->encoding;
    // const auto addr = ss.str();
    
    // RCLCPP_INFO(this->get_logger(), 
    //     "Received image: %dx%d, encoding: %s, address: %s",
    //     width, height, encoding.c_str(), addr.c_str());

    // sensor_msgs::msg::Image last_image_;
    // last_image_ = *msg;
    // image_publisher_->publish(std::move(*msg));
}

// void ImageSubscriberNode::demoCallback(std_msgs::msg::Int32::UniquePtr msg)
// {
//   std::stringstream ss;
//   ss << "0x" << std::hex << reinterpret_cast<std::uintptr_t>(msg.get());
  
//   RCLCPP_INFO(this->get_logger(), 
//     "Received demo message: %d, address: %s",
//     msg->data, ss.str().c_str());

//   std_msgs::msg::Int32 last_demo_;
//   last_demo_ = *msg;
//   demo_publisher_->publish(std::move(*msg));
  
// }

}  // namespace avt_vimba_camera


// Add these lines at the end of the file
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(avt_vimba_camera::ImageSubscriberNode)
