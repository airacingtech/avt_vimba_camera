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
    "/image/ptr",
    qos,
    std::bind(&ImageSubscriberNode::imageCallback, this, std::placeholders::_1),
    sub_options
  );


  // Create a publisher to re-publish the image on "output_image" topic
  // image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("ptr/output_image", 10);

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
    // const auto width = msg->width;
    // const auto height = msg->height;
    // const auto encoding = msg->encoding;
    // const auto addr = ss.str();
    
    // RCLCPP_INFO(this->get_logger(), 
    //     "Received image: %dx%d, encoding: %s, address: %s",
    //     width, height, encoding.c_str(), addr.c_str());
    try {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        
        // Create a named window first (with proper flags for ROS environment)
        std::string window_name = msg->header.frame_id.empty() ? 
            "Camera Image" : msg->header.frame_id;
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO);
        
        // Display the image
        cv::imshow(window_name, cv_ptr->image);
        cv::pollKey(); // Non-blocking key check instead of waitKey
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        return;
    }
}


}  // namespace avt_vimba_camera


// Add these lines at the end of the file
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(avt_vimba_camera::ImageSubscriberNode)
