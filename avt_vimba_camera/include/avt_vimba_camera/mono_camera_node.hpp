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

#ifndef MONO_CAMERA_H
#define MONO_CAMERA_H

#include "avt_vimba_camera/avt_vimba_camera.hpp"
#include "avt_vimba_camera/avt_vimba_api.hpp"

#include <avt_vimba_camera_msgs/srv/detail/load_settings__struct.hpp>
#include <avt_vimba_camera_msgs/srv/detail/save_settings__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>
#include "std_msgs/msg/int32.hpp"


namespace avt_vimba_camera
{
class MonoCameraNode : public rclcpp::Node
{
public:
  explicit MonoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MonoCameraNode();
  void start();

private:
  AvtVimbaApi api_;
  AvtVimbaCamera cam_;

  std::string ip_;
  std::string guid_;
  std::string camera_info_url_;
  std::string frame_id_;
  bool use_measurement_time_;
  bool publish_compressed_;
  int32_t ptp_offset_;

  image_transport::CameraPublisher camera_info_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub;
  std::shared_ptr<camera_info_manager::CameraInfoManager> info_man_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::weak_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> captured_pub;

  // std::weak_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> captured_pub_demo;
  // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_demo_;
  rclcpp::TimerBase::SharedPtr timer_demo_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

  rclcpp::Service<avt_vimba_camera_msgs::srv::LoadSettings>::SharedPtr load_srv_;
  rclcpp::Service<avt_vimba_camera_msgs::srv::SaveSettings>::SharedPtr save_srv_;

  void loadParams();
  void frameCallback(const FramePtr& vimba_frame_ptr);
  void publishImagePtr(sensor_msgs::msg::Image & image);

  void startSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std_srvs::srv::Trigger::Request::SharedPtr req,
                        std_srvs::srv::Trigger::Response::SharedPtr res);
  void stopSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                       const std_srvs::srv::Trigger::Request::SharedPtr req,
                       std_srvs::srv::Trigger::Response::SharedPtr res);
  void loadSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                       const avt_vimba_camera_msgs::srv::LoadSettings::Request::SharedPtr req,
                       avt_vimba_camera_msgs::srv::LoadSettings::Response::SharedPtr res);
  void saveSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                       const avt_vimba_camera_msgs::srv::SaveSettings::Request::SharedPtr req,
                       avt_vimba_camera_msgs::srv::SaveSettings::Response::SharedPtr res);
};
}  // namespace avt_vimba_camera
#endif
