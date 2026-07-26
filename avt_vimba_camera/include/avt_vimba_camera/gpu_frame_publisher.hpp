/// Copyright (c) 2026, AI Racing Tech

#ifndef AVT_VIMBA_CAMERA__GPU_FRAME_PUBLISHER_HPP_
#define AVT_VIMBA_CAMERA__GPU_FRAME_PUBLISHER_HPP_

#include <cuda_runtime.h>
#include <nppdefs.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <isaac_ros_managed_nitros/managed_nitros_publisher.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>

namespace avt_vimba_camera
{

class DeviceBufferPool
{
public:
  explicit DeviceBufferPool(size_t count);
  ~DeviceBufferPool();

  int Acquire(size_t bytes, void** data);
  void Release(int index);

private:
  struct Slot
  {
    void* data{ nullptr };
    size_t size{ 0 };
    bool in_use{ false };
  };

  std::mutex mutex_;
  std::vector<Slot> slots_;
};

class GpuFramePublisher
{
public:
  GpuFramePublisher(rclcpp::Node* node, const std::string& topic, size_t pool_size,
                    const std::string& scaled_topic, uint32_t scaled_long_edge);
  ~GpuFramePublisher();

  bool Publish(const std_msgs::msg::Header& header, const uint8_t* host_data, uint32_t width,
               uint32_t height, uint32_t step, const std::string& encoding);

private:
  bool Stage(const uint8_t* host_data, size_t bytes);

  rclcpp::Node* node_;
  cudaStream_t stream_{ nullptr };
  cudaEvent_t done_{ nullptr };
  NppStreamContext npp_ctx_{};

  void* staging_{ nullptr };
  size_t staging_size_{ 0 };

  std::shared_ptr<DeviceBufferPool> pool_;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> publisher_;

  void ResolveScaledSize(uint32_t width, uint32_t height);

  uint32_t scaled_long_edge_{ 0 };
  uint32_t source_width_{ 0 };
  uint32_t source_height_{ 0 };
  uint32_t scaled_width_{ 0 };
  uint32_t scaled_height_{ 0 };
  std::shared_ptr<DeviceBufferPool> scaled_pool_;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> scaled_publisher_;
};

}  // namespace avt_vimba_camera

#endif  // AVT_VIMBA_CAMERA__GPU_FRAME_PUBLISHER_HPP_
