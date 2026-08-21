/// Copyright (c) 2026, AI Racing Tech

#ifndef AVT_VIMBA_CAMERA__GPU_FRAME_PUBLISHER_HPP_
#define AVT_VIMBA_CAMERA__GPU_FRAME_PUBLISHER_HPP_

#include <cuda_runtime.h>
#include <nppdefs.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <isaac_ros_managed_nitros/managed_nitros_publisher.hpp>
#include <isaac_ros_nitros_image_type/nitros_image.hpp>

namespace avt_vimba_camera
{

/// One instrumented call site in the GPU frame path. Tracks wall time AND thread CPU time
/// separately: a site that blocks on the GPU (cudaEventSynchronize) has large wall time but near
/// zero CPU, and conflating the two makes a sleeping wait look like a hot spot.
struct ProfSite
{
  const char* name{ nullptr };
  std::atomic<uint64_t> wall_ns{ 0 };
  std::atomic<uint64_t> cpu_ns{ 0 };
  std::atomic<uint64_t> n{ 0 };
};

enum ProfId
{
  kProfRefreshSubs, kProfAcquire, kProfStage, kProfDebayer, kProfResize,
  kProfSync, kProfNitrosMain, kProfNitrosScaled, kProfTotal, kProfCount
};

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
                    const std::string& scaled_topic, uint32_t scaled_long_edge,
                    double scaled_max_fps = 0.0, double main_max_fps = 0.0);
  ~GpuFramePublisher();

  bool Publish(const std_msgs::msg::Header& header, const uint8_t* host_data, uint32_t width,
               uint32_t height, uint32_t step, const std::string& encoding);

  /// False only when nothing anywhere is consuming either NITROS stream. Publish() costs a
  /// full-frame PCIe upload plus a debayer, so with six cameras at full rate it is worth
  /// skipping entirely while unsubscribed. Transitions are logged, never silent.
  bool HasSubscribers();

private:
  bool Stage(const uint8_t* host_data, size_t bytes);

  /// The two streams are gated separately. In the common telemetry-only configuration nothing
  /// consumes the full-rate stream and only the 256 px scaled one is encoded, so treating them as
  /// one meant uploading and debayering every full-resolution frame for no consumer. Counts are
  /// cached because this runs per frame per camera and a graph query is not free.
  void RefreshSubscribers();
  bool main_subs_{ false };
  bool scaled_subs_{ false };
  std::chrono::steady_clock::time_point last_sub_check_{};

  rclcpp::Node* node_;
  cudaStream_t stream_{ nullptr };
  cudaEvent_t done_{ nullptr };
  NppStreamContext npp_ctx_{};

  void* staging_{ nullptr };
  size_t staging_size_{ 0 };

  /// Vimba recycles a small fixed set of frame buffers, so page-locking each one the first time
  /// it appears turns every subsequent cudaMemcpyAsync into a real DMA. Without this, CUDA has
  /// to bounce each frame through an internal pinned staging buffer, which costs an extra
  /// full-frame host memcpy per frame and cannot overlap with compute.
  std::unordered_set<const void*> pinned_;

  /// Resolved topic names, kept for count_subscribers().
  std::string topic_;
  std::string scaled_topic_;
  bool skipping_{ false };

  std::shared_ptr<DeviceBufferPool> pool_;
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> publisher_;

  void ResolveScaledSize(uint32_t width, uint32_t height);

  /// The scaled stream exists only to feed the H.264 telemetry uplink, whose rate controller is
  /// configured for a fixed framerate. Publishing it at full sensor rate makes the encoder spend
  /// its whole bitrate budget several times over AND burn CPU proportionally, so it is rate
  /// limited independently of the full-rate stream perception consumes. 0 means every frame.
  double scaled_max_fps_{ 0.0 };
  std::chrono::steady_clock::time_point last_scaled_{};

  /// Same idea for the full-rate stream. YOLOv8 is inference bound well below the sensor rate
  /// (measured ~11.7 Hz of detections against 37.7 Hz of frames delivered), so publishing every
  /// frame hands NITROS work to a consumer that discards most of it. 0 means every frame.
  double main_max_fps_{ 0.0 };
  std::chrono::steady_clock::time_point last_main_{};

  /// Last time the per-call-site profile was emitted (see the prof:: block in the .cpp).
  std::chrono::steady_clock::time_point last_prof_{};
  /// Per-INSTANCE, so each camera reports its own numbers. A shared static here would have all six
  /// cameras accumulating into one set of counters and draining each other's.
  ProfSite prof_[kProfCount];
  void ProfReport(double window_s);

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
