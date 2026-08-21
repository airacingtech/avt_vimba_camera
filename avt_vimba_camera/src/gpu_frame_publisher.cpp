/// Copyright (c) 2026, AI Racing Tech

#include "avt_vimba_camera/gpu_frame_publisher.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include <nppi_color_conversion.h>
#include <nppi_data_exchange_and_initialization.h>
#include <nppi_geometry_transforms.h>

#include <isaac_ros_nitros_image_type/nitros_image_builder.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace avt_vimba_camera
{

// ---------------------------------------------------------------------------------------------
// Per-call-site timing for the GPU frame path. perf is unusable on this box (perf_event_paranoid=4
// and no perf build for the -autobox kernel) and ptrace_scope=1 blocks attaching a profiler, so the
// hot path is instrumented directly. Each probe records wall time AND thread CPU time, because a
// site that blocks on the GPU has large wall time and almost no CPU -- reporting only wall would
// make a sleeping wait look like the hottest thing in the pipeline.
// Set AVT_VIMBA_PROFILE=0 to silence the reporting.
// ---------------------------------------------------------------------------------------------
namespace
{

uint64_t ThreadCpuNs()
{
  struct timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

class ProfScope
{
public:
  explicit ProfScope(ProfSite& s)
    : s_(s), wall0_(std::chrono::steady_clock::now()), cpu0_(ThreadCpuNs())
  {
  }
  ~ProfScope()
  {
    s_.wall_ns.fetch_add(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - wall0_).count()), std::memory_order_relaxed);
    s_.cpu_ns.fetch_add(ThreadCpuNs() - cpu0_, std::memory_order_relaxed);
    s_.n.fetch_add(1, std::memory_order_relaxed);
  }

private:
  ProfSite& s_;
  std::chrono::steady_clock::time_point wall0_;
  uint64_t cpu0_;
};

}  // namespace

void GpuFramePublisher::ProfReport(double window_s)
{
  static const char* kNames[kProfCount] = {
    "RefreshSubscribers", "pool.Acquire", "Stage(H2D enqueue)", "NPP debayer", "NPP resize",
    "cudaEventSynchronize", "NITROS build+publish", "NITROS scaled", "Publish() TOTAL"
  };
  std::string out;
  out.reserve(1024);
  for (int i = 0; i < kProfCount; ++i)
  {
    const uint64_t wall = prof_[i].wall_ns.exchange(0, std::memory_order_relaxed);
    const uint64_t cpu = prof_[i].cpu_ns.exchange(0, std::memory_order_relaxed);
    const uint64_t n = prof_[i].n.exchange(0, std::memory_order_relaxed);
    if (n == 0)
    {
      continue;
    }
    char buf[200];
    snprintf(buf, sizeof(buf),
             "\n    %-21s %6.1f/s  wall %7.1f us  cpu %7.1f us  -> CPU %5.2f%% of a core",
             kNames[i], n / window_s, wall / 1e3 / n, cpu / 1e3 / n,
             100.0 * cpu / 1e9 / window_s);
    out += buf;
  }
  if (!out.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "profile %.0fs:%s", window_s, out.c_str());
  }
}

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

uint32_t SnapToMacroblock(double value)
{
  const uint32_t snapped = static_cast<uint32_t>(std::lround(value / 16.0)) * 16;
  return std::max(snapped, 16u);
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
                                     size_t pool_size, const std::string& scaled_topic,
                                     uint32_t scaled_long_edge, double scaled_max_fps,
                                     double main_max_fps)
  : node_(node)
  , pool_(std::make_shared<DeviceBufferPool>(pool_size))
  , scaled_max_fps_(scaled_max_fps)
  , main_max_fps_(main_max_fps)
  , scaled_long_edge_(scaled_long_edge)
{
  cudaError_t error = cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
  if (error != cudaSuccess)
  {
    RCLCPP_WARN(node_->get_logger(), "cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync): %s",
                cudaGetErrorString(error));
  }

  error = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
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
      nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, rclcpp::QoS(1));
  topic_ = node_->get_node_topics_interface()->resolve_topic_name(topic);

  if (scaled_long_edge_ > 0)
  {
    scaled_pool_ = std::make_shared<DeviceBufferPool>(pool_size);
    scaled_publisher_ = std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosImage>>(
        node_, scaled_topic, nvidia::isaac_ros::nitros::nitros_image_bgr8_t::supported_type_name,
        nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig{}, rclcpp::QoS(1));
    scaled_topic_ = node_->get_node_topics_interface()->resolve_topic_name(scaled_topic);
    if (scaled_max_fps_ > 0.0)
    {
      RCLCPP_INFO(node_->get_logger(), "Scaled NITROS stream rate limited to %.1f fps",
                  scaled_max_fps_);
    }
  }
  if (main_max_fps_ > 0.0)
  {
    RCLCPP_INFO(node_->get_logger(), "Full-rate NITROS stream rate limited to %.1f fps",
                main_max_fps_);
  }
}

void GpuFramePublisher::RefreshSubscribers()
{
  // Cached: this is consulted per frame per camera (227/s across six at full rate) and a graph
  // query is not free. Half a second is far quicker than a consumer can matter.
  const auto now = std::chrono::steady_clock::now();
  if (last_sub_check_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_sub_check_) < std::chrono::duration<double>(0.5))
  {
    return;
  }
  last_sub_check_ = now;

  // NITROS type negotiation carries the payload on a "<topic>/nitros" companion topic, so a
  // negotiated consumer shows up there rather than on the base name. Count both, and treat any
  // error as "someone is listening" so a graph-query hiccup can never silently starve the path.
  try
  {
    main_subs_ = node_->count_subscribers(topic_) + node_->count_subscribers(topic_ + "/nitros") > 0;
    scaled_subs_ = !scaled_topic_.empty() &&
                   (node_->count_subscribers(scaled_topic_) +
                    node_->count_subscribers(scaled_topic_ + "/nitros")) > 0;
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_ONCE(node_->get_logger(), "count_subscribers failed (%s); always publishing",
                     e.what());
    main_subs_ = true;
    scaled_subs_ = scaled_publisher_ != nullptr;
  }
}

bool GpuFramePublisher::HasSubscribers()
{
  RefreshSubscribers();
  const bool has = main_subs_ || scaled_subs_;
  if (has == skipping_)
  {
    skipping_ = !has;
    RCLCPP_INFO(node_->get_logger(), has ?
                    "NITROS subscriber appeared; resuming GPU debayer/upload on %s" :
                    "No NITROS subscribers on %s; skipping GPU debayer/upload until one appears",
                node_->get_name());
  }
  return has;
}

GpuFramePublisher::~GpuFramePublisher()
{
  for (const void* p : pinned_)
  {
    cudaHostUnregister(const_cast<void*>(p));
  }
  pinned_.clear();

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

void GpuFramePublisher::ResolveScaledSize(uint32_t width, uint32_t height)
{
  if (width == source_width_ && height == source_height_)
  {
    return;
  }
  source_width_ = width;
  source_height_ = height;

  const double scale = static_cast<double>(scaled_long_edge_) / std::max(width, height);
  scaled_width_ = SnapToMacroblock(width * scale);
  scaled_height_ = SnapToMacroblock(height * scale);

  RCLCPP_INFO(node_->get_logger(), "Scaled NITROS stream: %ux%u -> %ux%u", width, height,
              scaled_width_, scaled_height_);
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

  // Page-lock this buffer the first time we see it. Vimba recycles a small fixed set of frame
  // buffers, so the set converges within the first few frames and every copy after that is a
  // true DMA instead of a bounce through CUDA's internal pinned staging area. Failure is
  // non-fatal: the copy still works, just via the slower pageable path.
  if (pinned_.find(host_data) == pinned_.end())
  {
    const cudaError_t reg =
        cudaHostRegister(const_cast<void*>(static_cast<const void*>(host_data)), bytes,
                         cudaHostRegisterDefault);
    if (reg == cudaSuccess)
    {
      pinned_.insert(host_data);
      RCLCPP_DEBUG(logger, "Page-locked Vimba frame buffer %p (%zu bytes); %zu pinned",
                   static_cast<const void*>(host_data), bytes, pinned_.size());
    }
    else
    {
      // Remember it anyway so we do not retry the same failing pointer every frame.
      pinned_.insert(host_data);
      RCLCPP_WARN_ONCE(logger, "cudaHostRegister failed (%s); frames stay on the pageable path",
                       cudaGetErrorString(reg));
      cudaGetLastError();
    }
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

  ProfScope prof_total(prof_[kProfTotal]);

  // Emit a profile window periodically. Cheap: one steady_clock read per frame.
  {
    static const bool enabled = [] {
      const char* e = std::getenv("AVT_VIMBA_PROFILE");
      return e == nullptr || std::string(e) != "0";
    }();
    if (enabled)
    {
      const auto now = std::chrono::steady_clock::now();
      if (last_prof_.time_since_epoch().count() == 0)
      {
        last_prof_ = now;
      }
      const double elapsed = std::chrono::duration<double>(now - last_prof_).count();
      if (elapsed >= 10.0)
      {
        last_prof_ = now;
        ProfReport(elapsed);
      }
    }
  }

  // Decide what this frame is actually for BEFORE spending a PCIe upload and a debayer on it.
  // In the telemetry-only configuration nothing consumes the full-rate stream and the scaled one
  // is rate limited, so most frames need neither and can be dropped here for free.
  {
    ProfScope s(prof_[kProfRefreshSubs]);
    RefreshSubscribers();
  }
  bool publish_main = main_subs_;
  if (publish_main && main_max_fps_ > 0.0)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / main_max_fps_);
    if (last_main_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - last_main_) < period)
    {
      publish_main = false;
    }
    else
    {
      last_main_ = now;
    }
  }
  bool want_scaled = scaled_publisher_ != nullptr && scaled_subs_;
  if (want_scaled && scaled_max_fps_ > 0.0)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / scaled_max_fps_);
    if (last_scaled_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - last_scaled_) < period)
    {
      want_scaled = false;
    }
    else
    {
      last_scaled_ = now;
    }
  }
  if (!publish_main && !want_scaled)
  {
    return true;
  }

  const int dst_step = static_cast<int>(width) * 3;
  void* out = nullptr;
  int slot;
  {
    ProfScope s(prof_[kProfAcquire]);
    slot = pool_->Acquire(static_cast<size_t>(dst_step) * height, &out);
  }
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

  bool staged;
  {
    ProfScope s(prof_[kProfStage]);
    if (is_bgr8)
    {
      failed = CudaFailed(logger, clock,
                          cudaMemcpy2DAsync(out, dst_step, host_data, step, dst_step, height,
                                            cudaMemcpyDefault, stream_),
                          "cudaMemcpy2DAsync");
      staged = false;
    }
    else
    {
      staged = Stage(host_data, static_cast<size_t>(step) * height);
      if (!staged)
      {
        failed = true;
      }
    }
  }

  if (staged)
  {
    ProfScope s(prof_[kProfDebayer]);
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

  void* scaled_out = nullptr;
  int scaled_slot = -1;
  int scaled_step = 0;
  if (!failed && want_scaled)
  {
    ProfScope s(prof_[kProfResize]);
    ResolveScaledSize(width, height);
    scaled_step = static_cast<int>(scaled_width_) * 3;
    scaled_slot =
        scaled_pool_->Acquire(static_cast<size_t>(scaled_step) * scaled_height_, &scaled_out);
    if (scaled_slot < 0)
    {
      RCLCPP_WARN_THROTTLE(logger, *clock, 2000,
                           "Every scaled GPU buffer is still in the encoder; dropping downscale");
    }
    else
    {
      const NppiRect src_roi{ 0, 0, static_cast<int>(width), static_cast<int>(height) };
      const NppiSize scaled_size{ static_cast<int>(scaled_width_),
                                  static_cast<int>(scaled_height_) };
      const NppiRect scaled_roi{ 0, 0, scaled_size.width, scaled_size.height };
      if (NppFailed(logger, clock,
                    nppiResize_8u_C3R_Ctx(dst, dst_step, size, src_roi,
                                          static_cast<Npp8u*>(scaled_out), scaled_step, scaled_size,
                                          scaled_roi, NPPI_INTER_LINEAR, npp_ctx_),
                    "nppiResize_8u_C3R_Ctx"))
      {
        scaled_pool_->Release(scaled_slot);
        scaled_slot = -1;
      }
    }
  }

  if (!failed)
  {
    ProfScope s(prof_[kProfSync]);
    failed = CudaFailed(logger, clock, cudaEventRecord(done_, stream_), "cudaEventRecord") ||
             CudaFailed(logger, clock, cudaEventSynchronize(done_), "cudaEventSynchronize");
  }

  if (failed)
  {
    pool_->Release(slot);
    if (scaled_slot >= 0)
    {
      scaled_pool_->Release(scaled_slot);
    }
    return false;
  }

  if (publish_main)
  {
    ProfScope s(prof_[kProfNitrosMain]);
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
      if (scaled_slot >= 0)
      {
        scaled_pool_->Release(scaled_slot);
        scaled_slot = -1;
      }
      RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "Could not publish NITROS image: %s", e.what());
      return false;
    }
  }
  else
  {
    // Nothing consumes the full-rate stream; the debayered buffer existed only as the resize
    // source, so hand it straight back instead of publishing to no one.
    pool_->Release(slot);
  }

  if (scaled_slot >= 0)
  {
    ProfScope s(prof_[kProfNitrosScaled]);
    try
    {
      nvidia::isaac_ros::nitros::NitrosImage scaled =
          nvidia::isaac_ros::nitros::NitrosImageBuilder()
              .WithHeader(header)
              .WithEncoding(sensor_msgs::image_encodings::BGR8)
              .WithDimensions(scaled_height_, scaled_width_)
              .WithGpuData(scaled_out)
              .WithReleaseCallback(
                  [pool = scaled_pool_, scaled_slot]() { pool->Release(scaled_slot); })
              .Build();
      scaled_publisher_->publish(scaled);
    }
    catch (const std::exception& e)
    {
      scaled_pool_->Release(scaled_slot);
      RCLCPP_ERROR_THROTTLE(logger, *clock, 2000, "Could not publish scaled NITROS image: %s",
                            e.what());
    }
  }

  return true;
}

}  // namespace avt_vimba_camera
