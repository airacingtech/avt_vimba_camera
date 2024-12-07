#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "avt_vimba_camera/mono_camera_node.hpp"
#include "avt_vimba_camera/image_subscriber_node.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    
    // Create node options with intra-process communication enabled
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    // Create both nodes with the same options
    auto camera_node = std::make_shared<avt_vimba_camera::MonoCameraNode>(options);
    auto subscriber_node = std::make_shared<avt_vimba_camera::ImageSubscriberNode>(options);

    // Create a single-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(camera_node);
    executor.add_node(subscriber_node);

    // Spin the executor
    executor.spin();

    // Cleanup
    camera_node.reset();
    subscriber_node.reset();
    rclcpp::shutdown();
    return 0;
} 