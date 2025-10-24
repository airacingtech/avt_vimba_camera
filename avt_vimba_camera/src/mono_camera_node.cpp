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

using namespace std::placeholders;
namespace avt_vimba_camera
{
MonoCameraNode::MonoCameraNode() : Node("camera"), api_(this->get_logger()), cam_(std::shared_ptr<rclcpp::Node>(dynamic_cast<rclcpp::Node * >(this)))
{
  
  camera_info_pub_ = image_transport::create_camera_publisher(this, "~/image");
  NitrosDiagnosticsConfig diag_config;
  nitros_img_pub_ = std::make_unique<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<nvidia::isaac_ros::nitros::NitrosImage>>(
        this,
        "~/image/nitros",
        "nitros_image_bgr8",
        diag_config,
        rclcpp::SensorDataQoS());
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCameraNode::frameCallback, this, _1));

  start_srv_ = create_service<std_srvs::srv::Trigger>("~/start_stream", std::bind(&MonoCameraNode::startSrvCallback, this, _1, _2, _3));
  stop_srv_ = create_service<std_srvs::srv::Trigger>("~/stop_stream", std::bind(&MonoCameraNode::stopSrvCallback, this, _1, _2, _3));

  load_srv_ = create_service<avt_vimba_camera_msgs::srv::LoadSettings>("~/load_settings", std::bind(&MonoCameraNode::loadSrvCallback, this, _1, _2, _3));
  save_srv_ = create_service<avt_vimba_camera_msgs::srv::SaveSettings>("~/save_settings", std::bind(&MonoCameraNode::saveSrvCallback, this, _1, _2, _3));

  loadParams();

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.reliable();

  if (publish_compressed_) {
    compressed_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("~/image/compressed", qos);
  }
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
  publish_compressed_ = this->declare_parameter("publish_compressed", true);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded");
}

void MonoCameraNode::start()
{
  // Start Vimba & list all available cameras
  api_.start();

  // Start camera
  cam_.start(ip_, guid_, frame_id_, camera_info_url_);
  cam_.startImaging();
}

void MonoCameraNode::frameCallback(const FramePtr& vimba_frame_ptr)
{
  static thread_local bool cuda_initialized = false;
  if (!cuda_initialized) {
    cudaSetDevice(0);
    cudaFree(0);
    cuda_initialized = true;
  }

  rclcpp::Time ros_time = this->get_clock()->now();

  sensor_msgs::msg::Image img;
  sensor_msgs::msg::CompressedImage compressed_image;
  if (!api_.frameToImage(vimba_frame_ptr, img, compressed_image, publish_compressed_)) {
    RCLCPP_WARN_STREAM(this->get_logger(), "frameToImage() failed. No image published.");
    return;
  }

  sensor_msgs::msg::CameraInfo ci = cam_.getCameraInfo();
  ci.header.frame_id = frame_id_;
  ci.header.stamp = ros_time;
  img.header = ci.header;
  camera_info_pub_.publish(img, ci);

  // ---- GPU Publishing ----
  size_t buffer_size{img.step * img.height};
  void * buffer;
  cudaMalloc(&buffer, buffer_size);

  // Copy data bytes to CUDA buffer
  cudaMemcpy(buffer, img.data.data(), buffer_size, cudaMemcpyDefault);

  if (nitros_img_pub_) {
    nvidia::isaac_ros::nitros::NitrosImage nitros_msg = NitrosImageBuilder()
      .WithHeader(img.header)
      .WithEncoding(img.encoding)
      .WithDimensions(img.height, img.width)
      .WithGpuData(buffer)
      .Build();

    nitros_img_pub_->publish(nitros_msg);
    RCLCPP_INFO(this->get_logger(), "Sent CUDA buffer with memory at: %p", buffer);
  }

  if (publish_compressed_) {
    compressed_image.header = ci.header;
    compressed_pub->publish(compressed_image);
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
