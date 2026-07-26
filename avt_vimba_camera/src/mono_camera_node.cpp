/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <thread>

#include <avt_vimba_camera/mono_camera_node.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/fill_image.hpp>

using namespace std::placeholders;

namespace avt_vimba_camera
{
MonoCameraNode::MonoCameraNode(const rclcpp::NodeOptions& options) : Node("camera", options), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
  // Set the image publisher before streaming
  camera_info_pub_ = image_transport::create_camera_publisher(this, "~/image");

  // Set the frame callback (for live camera)
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::frameCallback, this, _1));

  cam_.setPcapPublishCallback([this](const sensor_msgs::msg::CameraInfo& ci, const uint8_t* data,
                                     uint32_t width, uint32_t height, uint32_t step,
                                     const std::string& encoding) {
    publishFrame(ci, data, width, height, step, encoding);
  });

  start_srv_ = create_service<std_srvs::srv::Trigger>("~/start_stream", std::bind(&MonoCameraNode::startSrvCallback, this, _1, _2, _3));
  stop_srv_ = create_service<std_srvs::srv::Trigger>("~/stop_stream", std::bind(&MonoCameraNode::stopSrvCallback, this, _1, _2, _3));

  load_srv_ = create_service<avt_vimba_camera_msgs::srv::LoadSettings>("~/load_settings", std::bind(&MonoCameraNode::loadSrvCallback, this, _1, _2, _3));
  save_srv_ = create_service<avt_vimba_camera_msgs::srv::SaveSettings>("~/save_settings", std::bind(&MonoCameraNode::saveSrvCallback, this, _1, _2, _3));

  loadParams();

#ifdef AVT_VIMBA_CAMERA_WITH_NITROS
  if (use_gpu_pipeline_)
  {
    try
    {
      gpu_pub_ = std::make_unique<GpuFramePublisher>(
          this, "~/image/nitros", static_cast<size_t>(gpu_buffer_pool_size_),
          "~/image/nitros_scaled", static_cast<uint32_t>(scaled_long_edge_));
      RCLCPP_INFO(this->get_logger(), "Publishing NITROS device-memory frames on ~/image/nitros");
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "GPU pipeline unavailable (%s); frames stay on the host",
                   e.what());
      use_gpu_pipeline_ = false;
    }
  }
#else
  if (use_gpu_pipeline_)
  {
    RCLCPP_WARN(this->get_logger(),
                "use_gpu_pipeline is set but this driver was built without CUDA/NITROS");
    use_gpu_pipeline_ = false;
  }
#endif

  start();
}

MonoCameraNode::~MonoCameraNode()
{
  cam_.stop();
  camera_info_pub_.shutdown();
}

void MonoCameraNode::loadParams()
{
  ip_ = this->declare_parameter("ip", "");
  guid_ = this->declare_parameter("guid", "");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");
  frame_id_ = this->declare_parameter("frame_id", "");
  use_measurement_time_ = this->declare_parameter("use_measurement_time", false);
  ptp_offset_ = this->declare_parameter("ptp_offset", 0);

  rcl_interfaces::msg::ParameterDescriptor gpu_desc;
  gpu_desc.description =
      "Upload each frame straight into device memory and publish it as a NITROS image on "
      "~/image/nitros, so a composed Isaac ROS encode chain never sees a host-side copy. Ignored "
      "when the driver was built without CUDA/NITROS.";
#ifdef AVT_VIMBA_CAMERA_WITH_NITROS
  use_gpu_pipeline_ = this->declare_parameter("use_gpu_pipeline", true, gpu_desc);
#else
  use_gpu_pipeline_ = this->declare_parameter("use_gpu_pipeline", false, gpu_desc);
#endif
  gpu_buffer_pool_size_ = this->declare_parameter("gpu_buffer_pool_size", 4);

  rcl_interfaces::msg::ParameterDescriptor scaled_desc;
  scaled_desc.description =
      "Long edge, in pixels, of a second NITROS stream published on ~/image/nitros_scaled. The "
      "driver derives the other edge from the camera's detected geometry so the aspect ratio is "
      "preserved at any sensor resolution or decimation. 0 disables the stream.";
  scaled_long_edge_ = this->declare_parameter("scaled_long_edge", 0, scaled_desc);

  rcl_interfaces::msg::ParameterDescriptor pcap_enable_desc;
  pcap_enable_desc.description = "Enable PCAP replay mode instead of live camera streaming";
  enable_pcap_ = this->declare_parameter("enable_pcap", false, pcap_enable_desc);
  
  rcl_interfaces::msg::ParameterDescriptor pcap_file_desc;
  pcap_file_desc.description = "Path to PCAP file containing GigE Vision camera data for replay";
  pcap_file_ = this->declare_parameter("pcap_file", "", pcap_file_desc);

  this->declare_parameter("pcap_loop", false);
  this->declare_parameter("pcap_playback_speed", 1.0);
}

void MonoCameraNode::start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_, enable_pcap_, pcap_file_);
  cam_.startImaging();
}

void MonoCameraNode::frameCallback(const FramePtr& vimba_frame_ptr)
{
  rclcpp::Time ros_time = this->get_clock()->now();

  AvtVimbaApi::RawFrame frame;
  if (!api_.describeFrame(vimba_frame_ptr, frame))
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Could not describe frame. No image published.");
    return;
  }

  sensor_msgs::msg::CameraInfo ci = cam_.getCameraInfo();
  // Note: getCameraInfo() doesn't fill in header frame_id or stamp
  ci.header.frame_id = frame_id_;
  ci.header.stamp = ros_time;

  publishFrame(ci, frame.data, frame.width, frame.height, frame.step, frame.encoding);
}

void MonoCameraNode::publishFrame(const sensor_msgs::msg::CameraInfo& ci, const uint8_t* data,
                                  uint32_t width, uint32_t height, uint32_t step,
                                  const std::string& encoding)
{
#ifdef AVT_VIMBA_CAMERA_WITH_NITROS
  if (gpu_pub_)
  {
    gpu_pub_->Publish(ci.header, data, width, height, step, encoding);
  }
#endif

  if (camera_info_pub_.getNumSubscribers() > 0)
  {
    sensor_msgs::msg::Image img;
    img.header = ci.header;
    sensor_msgs::fillImage(img, encoding, height, width, step, data);
    camera_info_pub_.publish(img, ci);
  }
}

void MonoCameraNode::startSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                      const std_srvs::srv::Trigger::Request::SharedPtr req,
                                      std_srvs::srv::Trigger::Response::SharedPtr res) {
  (void)request_header;
  (void)req;

  cam_.startImaging();
  cam_.setForceStop(false);
  auto state = cam_.getCameraState();
  res->success = state != CameraState::ERROR;
}

void MonoCameraNode::stopSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std_srvs::srv::Trigger::Request::SharedPtr req,
                                     std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)request_header;
  (void)req;

  cam_.stopImaging();
  cam_.setForceStop(true);
  auto state = cam_.getCameraState();
  res->success = state != CameraState::ERROR;
}

void MonoCameraNode::loadSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const avt_vimba_camera_msgs::srv::LoadSettings::Request::SharedPtr req,
                                     avt_vimba_camera_msgs::srv::LoadSettings::Response::SharedPtr res)
{
  (void)request_header;
  auto extension = req->input_path.substr(req->input_path.find_last_of(".") + 1);
  if (extension != "xml")
  {
    RCLCPP_WARN(this->get_logger(), "Invalid file extension. Only .xml is supported.");
    res->result = false;
  }
  else
  {
    res->result = cam_.loadCameraSettings(req->input_path);
  }
}

void MonoCameraNode::saveSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const avt_vimba_camera_msgs::srv::SaveSettings::Request::SharedPtr req,
                                     avt_vimba_camera_msgs::srv::SaveSettings::Response::SharedPtr res)
{
  (void)request_header;
  auto extension = req->output_path.substr(req->output_path.find_last_of(".") + 1);
  if (extension != "xml")
  {
    RCLCPP_WARN(this->get_logger(), "Invalid file extension. Only .xml is supported.");
    res->result = false;
  }
  else
  {
    res->result = cam_.saveCameraSettings(req->output_path);
  }
}

}  // namespace avt_vimba_camera

RCLCPP_COMPONENTS_REGISTER_NODE(avt_vimba_camera::MonoCameraNode)