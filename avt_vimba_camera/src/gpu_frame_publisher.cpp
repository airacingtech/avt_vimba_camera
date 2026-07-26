/// Copyright (c) 2026, AI Racing Tech

#include "avt_vimba_camera/gpu_frame_publisher.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

#include <nppi_color_conversion.h>
#include <nppi_data_exchange_and_initialization.h>

#include <isaac_ros_nitros_image_type/nitros_image_builder.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace avt_vimba_camera
{
namespace
{

const std::map<std::string, NppiBayerGridPosition> kBayerGrids{
  { sensor_msgs::image_encodings::BAYER_RGGB8, NPPI_BAYER_RGGB },
  { sensor_msgs::image_encodings::BAYER_GRBG8, NPPI_BAYER_GRBG },
  { sensor_msgs::image_encodings::BAYER_GBRG8, NPPI_BAYER_GBRG },
  { sensor_msgs::image_encodings::BAYER_BGGR8, NPPI_BAYER_BGGR },
};

bool CudaFailed(const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock, cudaError_t error,
                const char* what)
{
  if (error == cudaSuccess)
  {
    return false;
  }
  RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "%s: %s", what, cudaGetErrorString(error));
  return true;
}

bool NppFailed(const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock, NppStatus status,
               const char* what)
{
  if (status == NPP_SUCCESS)
  {
    return false;
  }
  RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "%s: NppStatus %d", what, static_cast<int>(status));
  return true;
}

}  // namespace

DeviceBufferPool::DeviceBufferPool(size_t count) : slots_(count)
{
}

DeviceBufferPool::~DeviceBufferPool()
{
  for (auto& slot : slots_)
  {
    if (slot.data != nullptr)
    {
      cudaFree(slot.data);
    }
  }
}

int DeviceBufferPool::Acquire(size_t bytes, void** data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < slots_.size(); ++i)
  {
    Slot& slot = slots_[i];
    if (slot.in_use)
    {
      continue;
    }
    if (slot.size < bytes)
    {
      if (slot.data != nullptr)
      {
        cudaFree(slot.data);
        slot.data = nullptr;
        slot.size = 0;
      }
      if (cudaMalloc(&slot.data, bytes) != cudaSuccess)
      {
        slot.data = nullptr;
        return -1;
      }
      slot.size = bytes;
    }
    slot.in_use = true;
    *data = slot.data;
    return static_cast<int>(i);
  }
  return -1;
}

void DeviceBufferPool::Release(int index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  slots_[static_cast<size_t>(index)].in_use = false;
}

GpuFramePublisher::GpuFramePublisher(rclcpp::Node* node, const std::string& topic,
                                     size_t pool_size)
  : node_(node), pool_(std::make_shared<DeviceBufferPool>(pool_size))
{
  cudaError_t error = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  if (error != cudaSuccess)
  {
    throw std::runtime_error(std::string("cudaStreamCreateWithFlags failed: ") +
                             cudaGetErrorString(error));
  }

  error = cudaEventCreateWithFlags(&done_, cudaEventBlockingSync | cudaEventDisableTiming);
  if (error != cudaSuccess)
  {
    cudaStreamDestroy(stream_);
    throw std::runtime_error(std::string("cudaEventCreateWithFlags failed: ") +
                             cudaGetErrorString(error));
  }

  int device = 0;
  cudaGetDevice(&device);
  cudaDeviceProp props{};
  cudaGetDeviceProperties(&props, device);

  npp_ctx_.hStream = stream_;
  npp_ctx_.nCudaDeviceId = device;
  npp_ctx_.nMultiProcessorCount = props.multiProcessorCount;
  npp_ctx_.nMaxThreadsPerMultiProcessor = props.maxThreadsPerMultiProcessor;
  npp_ctx_.nMaxThreadsPerBlock = props.maxThreadsPerBlock;
  npp_ctx_.nSharedMemPerBlock = props.sharedMemPerBlock;
  npp_ctx_.nCudaDevAttrComputeCapabilityMajor = props.major;
  npp_ctx_.nCudaDevAttrComputeCapabilityMinor = props.minor;
  cudaStreamGetFlags(stream_, &npp_ctx_.nStreamFlags);

  publisher_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>>(
      node_, topic, nvidia::isaac_ros::nitros::nitros_image_bgr8_t::supported_type_name,
      nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, rclcpp::QoS(10));
}

GpuFramePublisher::~GpuFramePublisher()
{
  if (staging_ != nullptr)
  {
    cudaFree(staging_);
  }
  if (done_ != nullptr)
  {
    cudaEventDestroy(done_);
  }
  if (stream_ != nullptr)
  {
    cudaStreamDestroy(stream_);
  }
}

bool GpuFramePublisher::Stage(const uint8_t* host_data, size_t bytes)
{
  const rclcpp::Logger logger = node_->get_logger();
  rclcpp::Clock::SharedPtr clock = node_->get_clock();

  if (staging_size_ < bytes)
  {
    if (staging_ != nullptr)
    {
      cudaFree(staging_);
      staging_ = nullptr;
      staging_size_ = 0;
    }
    if (CudaFailed(logger, clock, cudaMalloc(&staging_, bytes), "cudaMalloc"))
    {
      staging_ = nullptr;
      return false;
    }
    staging_size_ = bytes;
  }

  return !CudaFailed(
      logger, clock,
      cudaMemcpyAsync(staging_, host_data, bytes, cudaMemcpyDefault, stream_),
      "cudaMemcpyAsync");
}

bool GpuFramePublisher::Publish(const std_msgs::msg::Header& header, const uint8_t* host_data,
                                uint32_t width, uint32_t height, uint32_t step,
                                const std::string& encoding)
{
  const rclcpp::Logger logger = node_->get_logger();
  rclcpp::Clock::SharedPtr clock = node_->get_clock();

  if (width % 2 != 0 || height % 2 != 0)
  {
    RCLCPP_ERROR_ONCE(logger, "%ux%u cannot go on the GPU path; NITROS needs even dimensions",
                      width, height);
    return false;
  }

  const auto grid = kBayerGrids.find(encoding);
  const bool is_rgb8 = encoding == sensor_msgs::image_encodings::RGB8;
  const bool is_bgr8 = encoding == sensor_msgs::image_encodings::BGR8;
  const bool is_mono8 = encoding == sensor_msgs::image_encodings::MONO8;
  if (grid == kBayerGrids.end() && !is_rgb8 && !is_bgr8 && !is_mono8)
  {
    RCLCPP_ERROR_ONCE(logger, "GPU path does not handle encoding '%s'", encoding.c_str());
    return false;
  }

  const int dst_step = static_cast<int>(width) * 3;
  void* out = nullptr;
  const int slot = pool_->Acquire(static_cast<size_t>(dst_step) * height, &out);
  if (slot < 0)
  {
    RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
                         "Every GPU frame buffer is still in the encoder; dropping frame");
    return false;
  }

  const NppiSize size{ static_cast<int>(width), static_cast<int>(height) };
  auto* dst = static_cast<Npp8u*>(out);
  bool failed = false;

  static const int kReverseRgb[3] = { 2, 1, 0 };

  if (is_bgr8)
  {
    failed = CudaFailed(logger, clock,
                        cudaMemcpy2DAsync(out, dst_step, host_data, step, dst_step, height,
                                          cudaMemcpyDefault, stream_),
                        "cudaMemcpy2DAsync");
  }
  else if (!Stage(host_data, static_cast<size_t>(step) * height))
  {
    failed = true;
  }
  else
  {
    const auto* src = static_cast<const Npp8u*>(staging_);
    if (grid != kBayerGrids.end())
    {
      const NppiRect roi{ 0, 0, static_cast<int>(width), static_cast<int>(height) };
      failed = NppFailed(logger, clock,
                         nppiCFAToRGB_8u_C1C3R_Ctx(src, static_cast<int>(step), size, roi, dst,
                                                   dst_step, grid->second, NPPI_INTER_UNDEFINED,
                                                   npp_ctx_),
                         "nppiCFAToRGB_8u_C1C3R_Ctx") ||
               NppFailed(logger, clock,
                         nppiSwapChannels_8u_C3IR_Ctx(dst, dst_step, size, kReverseRgb, npp_ctx_),
                         "nppiSwapChannels_8u_C3IR_Ctx");
    }
    else if (is_rgb8)
    {
      failed = NppFailed(logger, clock,
                         nppiSwapChannels_8u_C3R_Ctx(src, static_cast<int>(step), dst, dst_step,
                                                     size, kReverseRgb, npp_ctx_),
                         "nppiSwapChannels_8u_C3R_Ctx");
    }
    else
    {
      failed = NppFailed(logger, clock,
                         nppiDup_8u_C1C3R_Ctx(src, static_cast<int>(step), dst, dst_step, size,
                                              npp_ctx_),
                         "nppiDup_8u_C1C3R_Ctx");
    }
  }

  if (!failed)
  {
    failed = CudaFailed(logger, clock, cudaEventRecord(done_, stream_), "cudaEventRecord") ||
             CudaFailed(logger, clock, cudaEventSynchronize(done_), "cudaEventSynchronize");
  }

  if (failed)
  {
    pool_->Release(slot);
    return false;
  }

  try
  {
    nvidia::isaac_ros::nitros::NitrosImage image =
        nvidia::isaac_ros::nitros::NitrosImageBuilder()
            .WithHeader(header)
            .WithEncoding(sensor_msgs::image_encodings::BGR8)
            .WithDimensions(height, width)
            .WithGpuData(out)
            .WithReleaseCallback([pool = pool_, slot]() { pool->Release(slot); })
            .Build();
    publisher_->publish(image);
  }
  catch (const std::exception& e)
  {
    pool_->Release(slot);
    RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "Could not publish NITROS image: %s", e.what());
    return false;
  }

  return true;
}

}  // namespace avt_vimba_camera
