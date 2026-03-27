/**
 * CUDA zero-copy camera driver for Allied Vision cameras via VmbC API.
 *
 * Drop-in alternative for avt_vimba_camera mono_camera_node with:
 *   - Pinned (page-locked) DMA buffers announced directly to Vimba
 *   - Lock-free SPSC queue between Vimba callback and publisher thread
 *   - GPU Malvar-He-Cutler debayering (no CPU demosaic)
 *
 * Externally identical: same topics, parameters, and services as the
 * original avt_vimba_camera driver.
 */

#include "avt_vimba_camera/cuda_camera_node.hpp"
#include "avt_vimba_camera/pinned_buffer_pool.hpp"
#include "avt_vimba_camera/spsc_queue.hpp"

#include <VmbC/VmbC.h>

#include <cuda_runtime.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Forward-declare the CUDA debayer function defined in debayer.cu
namespace avt_vimba_camera {
extern void cuda_debayer(
    const uint8_t* bayer_in,
    uint8_t* rgb_out,
    int width, int height,
    cudaStream_t stream);
}  // namespace avt_vimba_camera

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
namespace {

/// Convert VmbError_t to human-readable string
const char* vmb_error_str(VmbError_t err) {
    switch (err) {
        case VmbErrorSuccess:           return "Success";
        case VmbErrorInternalFault:     return "InternalFault";
        case VmbErrorApiNotStarted:     return "ApiNotStarted";
        case VmbErrorNotFound:          return "NotFound";
        case VmbErrorBadHandle:         return "BadHandle";
        case VmbErrorDeviceNotOpen:     return "DeviceNotOpen";
        case VmbErrorInvalidAccess:     return "InvalidAccess";
        case VmbErrorBadParameter:      return "BadParameter";
        case VmbErrorStructSize:        return "StructSize";
        case VmbErrorMoreData:          return "MoreData";
        case VmbErrorNotImplemented:    return "NotImplemented";
        case VmbErrorNotSupported:      return "NotSupported";
        case VmbErrorResources:         return "Resources";
        case VmbErrorAlready:           return "Already";
        case VmbErrorNoData:            return "NoData";
        case VmbErrorInvalidValue:      return "InvalidValue";
        case VmbErrorTimeout:           return "Timeout";
        case VmbErrorOther:             return "Other";
        default:                        return "Unknown";
    }
}

/// Macro for concise VmbC error checking with logging
#define VMB_CHECK(call, logger, msg)                                        \
    do {                                                                     \
        VmbError_t _err = (call);                                           \
        if (_err != VmbErrorSuccess) {                                      \
            RCLCPP_ERROR(logger, "%s: %s (VmbError %d)",                    \
                         msg, vmb_error_str(_err), static_cast<int>(_err)); \
            return;                                                          \
        }                                                                    \
    } while (0)

/// Macro variant that returns false on failure (for bool-returning functions)
#define VMB_CHECK_BOOL(call, logger, msg)                                   \
    do {                                                                     \
        VmbError_t _err = (call);                                           \
        if (_err != VmbErrorSuccess) {                                      \
            RCLCPP_ERROR(logger, "%s: %s (VmbError %d)",                    \
                         msg, vmb_error_str(_err), static_cast<int>(_err)); \
            return false;                                                    \
        }                                                                    \
    } while (0)

/// Maximum number of camera open retries (E5)
constexpr int kCameraOpenMaxRetries = 30;
/// Interval between retries in seconds (E5)
constexpr int kCameraOpenRetryIntervalSec = 2;

}  // anonymous namespace

namespace avt_vimba_camera {

// ============================================================================
// Constructor
// ============================================================================
CudaCameraNode::CudaCameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("cuda_camera_node", options),
      camera_handle_(nullptr),
      camera_open_(false),
      d_rgb_output_(nullptr),
      cuda_stream_(nullptr),
      running_(false)
{
    // ---- Declare parameters (matching avt_vimba_camera interface) ----

    // Connection parameters
    this->declare_parameter<std::string>("ip", "");
    this->declare_parameter<std::string>("guid", "");
    this->declare_parameter<std::string>("camera_info_url", "");
    this->declare_parameter<std::string>("frame_id", "camera");
    this->declare_parameter<int>("ptp_offset", 0);
    this->declare_parameter<bool>("use_ptp", false);
    this->declare_parameter<bool>("publish_compressed", false);
    this->declare_parameter<int>("jpeg_quality", 80);
    this->declare_parameter<int>("num_buffers", 6);
    this->declare_parameter<bool>("gpu_direct", false);
    this->declare_parameter<bool>("enable_pcap", false);
    this->declare_parameter<std::string>("pcap_file", "");

    // --- v2.0.0 configurable optimizations ---
    // Raw Bayer: skip debayer in driver, publish BayerRG8 (3x smaller msgs).
    // Perception ISP handles debayer+undistort+resize in one JAX kernel.
    this->declare_parameter<bool>("publish_raw_bayer", true);

    // ROI crop: eliminate top portion of image (sky) on-sensor.
    // Camera only reads out the ROI → higher max frame rate.
    // Default: crop top 1/3 of 1544 = skip 515 rows → 2064x1029 output.
    this->declare_parameter<bool>("roi_enabled", false);
    this->declare_parameter<int>("roi_offset_x", 0);
    this->declare_parameter<int>("roi_offset_y", 515);
    this->declare_parameter<int>("roi_width", 0);   // 0 = full sensor width
    this->declare_parameter<int>("roi_height", 0);  // 0 = auto (sensor_height - offset_y)

    // PTP hardware sync: sub-microsecond timestamp alignment across cameras.
    // Requires PTP-capable Ethernet switch. Falls back to system clock if unavailable.
    this->declare_parameter<bool>("enable_ptp_sync", true);

    // Acquisition parameters (feature/* namespace, matching original driver)
    this->declare_parameter<double>("feature/frame_rate", 30.0);
    this->declare_parameter<std::string>("feature/exposure_auto", "Continuous");
    this->declare_parameter<double>("feature/exposure_time", 10000.0);
    this->declare_parameter<std::string>("feature/exposure_auto_target", "50");
    this->declare_parameter<std::string>("feature/gain_auto", "Continuous");
    this->declare_parameter<double>("feature/gain", 0.0);
    this->declare_parameter<std::string>("feature/balance_white_auto", "Continuous");
    this->declare_parameter<int>("feature/width", 2064);
    this->declare_parameter<int>("feature/height", 1544);
    this->declare_parameter<int>("feature/offset_x", 0);
    this->declare_parameter<int>("feature/offset_y", 0);
    this->declare_parameter<std::string>("feature/pixel_format", "BayerRG8");
    this->declare_parameter<std::string>("feature/trigger_source", "FixedRate");
    this->declare_parameter<std::string>("feature/trigger_mode", "On");
    this->declare_parameter<std::string>("feature/trigger_selector", "FrameStart");
    this->declare_parameter<double>("feature/balance_ratio_red", 1.0);
    this->declare_parameter<double>("feature/balance_ratio_blue", 1.0);
    this->declare_parameter<std::string>("feature/stream_bytes_per_second", "115000000");

    // Read back parameters into member variables
    camera_id_ = this->get_parameter("ip").as_string();
    if (camera_id_.empty()) {
        camera_id_ = this->get_parameter("guid").as_string();
    }
    camera_info_url_ = this->get_parameter("camera_info_url").as_string();
    frame_rate_ = this->get_parameter("feature/frame_rate").as_double();
    num_buffers_ = this->get_parameter("num_buffers").as_int();
    width_ = static_cast<uint32_t>(this->get_parameter("feature/width").as_int());
    height_ = static_cast<uint32_t>(this->get_parameter("feature/height").as_int());

    // Cache PTP parameters for callback thread (D6 fix)
    use_ptp_.store(this->get_parameter("use_ptp").as_bool(), std::memory_order_relaxed);
    ptp_offset_.store(this->get_parameter("ptp_offset").as_int(), std::memory_order_relaxed);

    // v2.0.0 optimization flags
    publish_raw_bayer_ = this->get_parameter("publish_raw_bayer").as_bool();
    roi_enabled_ = this->get_parameter("roi_enabled").as_bool();
    enable_ptp_sync_ = this->get_parameter("enable_ptp_sync").as_bool();

    if (roi_enabled_) {
        int roi_oy = this->get_parameter("roi_offset_y").as_int();
        int roi_h = this->get_parameter("roi_height").as_int();
        if (roi_h <= 0) roi_h = static_cast<int>(height_) - roi_oy;
        int roi_ox = this->get_parameter("roi_offset_x").as_int();
        int roi_w = this->get_parameter("roi_width").as_int();
        if (roi_w <= 0) roi_w = static_cast<int>(width_);
        // Apply ROI to effective dimensions
        width_ = static_cast<uint32_t>(roi_w);
        height_ = static_cast<uint32_t>(roi_h);
        RCLCPP_INFO(this->get_logger(),
            "ROI crop enabled: offset=(%d,%d) size=%dx%d (was %dx%d)",
            roi_ox, roi_oy, roi_w, roi_h,
            this->get_parameter("feature/width").as_int(),
            this->get_parameter("feature/height").as_int());
    }

    RCLCPP_INFO(this->get_logger(),
        "Mode: %s | ROI: %s | PTP: %s | GPU-direct: %s",
        publish_raw_bayer_ ? "raw_bayer" : "rgb8",
        roi_enabled_ ? "on" : "off",
        enable_ptp_sync_ ? "on" : "off",
        this->get_parameter("gpu_direct").as_bool() ? "on" : "off");

    // ---- Create image_transport CameraPublisher (fix A: topic compatibility) ----
    camera_pub_ = image_transport::create_camera_publisher(this, "~/image");

    // ---- Camera info manager ----
    camera_info_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, this->get_name(), camera_info_url_);

    // ---- Services (matching avt_vimba_camera: fix C) ----
    start_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/start_stream",
        std::bind(&CudaCameraNode::on_start_stream, this,
                  std::placeholders::_1, std::placeholders::_2));
    stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_stream",
        std::bind(&CudaCameraNode::on_stop_stream, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ---- Stub load/save settings services (fix #10) ----
    load_srv_ = this->create_service<avt_vimba_camera_msgs::srv::LoadSettings>(
        "~/load_settings",
        std::bind(&CudaCameraNode::on_load_settings, this,
                  std::placeholders::_1, std::placeholders::_2));
    save_srv_ = this->create_service<avt_vimba_camera_msgs::srv::SaveSettings>(
        "~/save_settings",
        std::bind(&CudaCameraNode::on_save_settings, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ---- Parameter change callback ----
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CudaCameraNode::on_parameter_change, this, std::placeholders::_1));

    // ---- Initialize CUDA stream ----
    cudaStream_t stream;
    cudaError_t cuda_err = cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
    if (cuda_err != cudaSuccess) {
        RCLCPP_ERROR(this->get_logger(), "cudaStreamCreate failed: %s",
                     cudaGetErrorString(cuda_err));
        throw std::runtime_error("CUDA stream creation failed");
    }
    cuda_stream_ = static_cast<void*>(stream);

    // ---- Allocate GPU output buffers ----
    const size_t rgb_size = static_cast<size_t>(width_) * height_ * 3;

    cuda_err = cudaMalloc(&d_rgb_output_, rgb_size);
    if (cuda_err != cudaSuccess) {
        RCLCPP_ERROR(this->get_logger(), "cudaMalloc(rgb) failed: %s",
                     cudaGetErrorString(cuda_err));
        throw std::runtime_error("CUDA rgb allocation failed");
    }

    // ---- GPU-direct zero-copy frame sharing ----
    gpu_direct_enabled_ = this->get_parameter("gpu_direct").as_bool();
    if (gpu_direct_enabled_) {
        for (int i = 0; i < kMaxGpuDirectBuffers; ++i) {
            cuda_err = cudaMalloc(reinterpret_cast<void**>(&d_gpu_direct_bufs_[i]), rgb_size);
            if (cuda_err != cudaSuccess) {
                RCLCPP_ERROR(this->get_logger(),
                             "cudaMalloc(gpu_direct[%d]) failed: %s",
                             i, cudaGetErrorString(cuda_err));
                throw std::runtime_error("CUDA gpu_direct allocation failed");
            }
            gpu_direct_slot_held_[i].store(false, std::memory_order_relaxed);
        }
        RCLCPP_INFO(this->get_logger(),
                     "GPU-direct zero-copy enabled with %d ring buffers",
                     kMaxGpuDirectBuffers);
    }

    // ---- Initialize pinned buffer pool ----
    // Each BayerRG8 frame is width * height bytes
    const size_t bayer_frame_size = static_cast<size_t>(width_) * height_;
    buffer_pool_ = std::make_unique<PinnedBufferPool>(
        static_cast<size_t>(num_buffers_), bayer_frame_size);

    // ---- Initialize SPSC queue ----
    frame_queue_ = std::make_unique<SPSCQueue<FrameEntry>>(
        static_cast<size_t>(num_buffers_));

    // ---- Read PCAP parameters ----
    enable_pcap_ = this->get_parameter("enable_pcap").as_bool();
    pcap_file_path_ = this->get_parameter("pcap_file").as_string();

    // ---- Open camera and start streaming ----
    if (enable_pcap_) {
        // GAP 1: PCAP replay mode -- skip VmbC camera open
        if (pcap_file_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "PCAP mode enabled but no pcap_file provided");
            return;
        }
        pcap_reader_ = std::make_shared<PcapReader>(
            pcap_file_path_, camera_id_, this->get_logger());
        if (!pcap_reader_->open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PCAP file: '%s'",
                         pcap_file_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "PCAP replay mode active: %s",
                     pcap_file_path_.c_str());
        start_pcap_replay();
    } else {
        RCLCPP_INFO(this->get_logger(), "Initializing VmbC API...");
        open_camera();

        if (camera_open_) {
            discover_camera_features();   // GAP 2: enumerate all features
            configure_camera_features();
            start_capture();

            // GAP 3: Start watchdog timer (1 Hz)
            watchdog_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&CudaCameraNode::watchdog_callback, this));
        }
    }
}

// ============================================================================
// Destructor
// ============================================================================
CudaCameraNode::~CudaCameraNode()
{
    // Stop watchdog timer
    if (watchdog_timer_) {
        watchdog_timer_->cancel();
        watchdog_timer_.reset();
    }

    // Stop PCAP threads
    if (pcap_thread_running_) {
        pcap_thread_running_.store(false);
        keyboard_thread_running_.store(false);
        if (pcap_thread_.joinable()) pcap_thread_.join();
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        restore_terminal();
    }
    if (pcap_reader_) {
        pcap_reader_->close();
    }

    // Signal publisher thread to stop
    running_.store(false, std::memory_order_release);
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }

    // Stop camera acquisition and close
    if (camera_open_) {
        stop_capture();
        close_camera();
    }

    // Free CUDA resources
    if (d_rgb_output_) {
        cudaFree(d_rgb_output_);
        d_rgb_output_ = nullptr;
    }
    for (int i = 0; i < kMaxGpuDirectBuffers; ++i) {
        if (d_gpu_direct_bufs_[i]) {
            cudaFree(d_gpu_direct_bufs_[i]);
            d_gpu_direct_bufs_[i] = nullptr;
        }
    }
    if (cuda_stream_) {
        cudaStreamDestroy(static_cast<cudaStream_t>(cuda_stream_));
        cuda_stream_ = nullptr;
    }

    // buffer_pool_ and frame_queue_ cleaned up by unique_ptr
}

// ============================================================================
// GPU-direct zero-copy frame API
// ============================================================================
std::optional<GpuFrame> CudaCameraNode::get_latest_gpu_frame()
{
    std::lock_guard<std::mutex> lock(gpu_frame_mutex_);
    if (!latest_gpu_frame_.has_value()) {
        return std::nullopt;
    }
    // Mark the slot as held so the writer skips it
    int idx = latest_gpu_frame_->buffer_index;
    gpu_direct_slot_held_[idx].store(true, std::memory_order_release);
    return latest_gpu_frame_;
}

void CudaCameraNode::release_gpu_frame(int buffer_index)
{
    if (buffer_index >= 0 && buffer_index < kMaxGpuDirectBuffers) {
        gpu_direct_slot_held_[buffer_index].store(false, std::memory_order_release);
    }
}

// ============================================================================
// Camera open / close
// ============================================================================
void CudaCameraNode::open_camera()
{
    // Start VmbC
    VmbError_t err = VmbStartup(nullptr);
    if (err != VmbErrorSuccess) {
        RCLCPP_ERROR(this->get_logger(), "VmbStartup failed: %s (%d)",
                     vmb_error_str(err), static_cast<int>(err));
        return;
    }

    // Open camera by IP or GUID, with retry loop (fix E5)
    RCLCPP_INFO(this->get_logger(), "Opening camera: %s", camera_id_.c_str());

    VmbAccessMode_t access_mode = VmbAccessModeFull;
    for (int attempt = 1; attempt <= kCameraOpenMaxRetries; ++attempt) {
        err = VmbCameraOpen(camera_id_.c_str(), access_mode, &camera_handle_);
        if (err == VmbErrorSuccess) {
            break;
        }
        RCLCPP_WARN(this->get_logger(),
                     "VmbCameraOpen('%s') attempt %d/%d failed: %s (%d) -- retrying in %ds",
                     camera_id_.c_str(), attempt, kCameraOpenMaxRetries,
                     vmb_error_str(err), static_cast<int>(err),
                     kCameraOpenRetryIntervalSec);
        if (attempt == kCameraOpenMaxRetries) {
            RCLCPP_ERROR(this->get_logger(),
                         "VmbCameraOpen('%s') failed after %d attempts, giving up",
                         camera_id_.c_str(), kCameraOpenMaxRetries);
            VmbShutdown();
            return;
        }
        std::this_thread::sleep_for(
            std::chrono::seconds(kCameraOpenRetryIntervalSec));
    }

    camera_open_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera opened successfully: %s", camera_id_.c_str());

    // ---- GigE packet size optimization (fix E3) ----
    // Negotiate the largest possible packet size on the network path.
    // This is a best-effort command; non-GigE cameras will return NotSupported.
    VmbError_t pkt_err = VmbFeatureCommandRun(camera_handle_, "GVSPAdjustPacketSize");
    if (pkt_err == VmbErrorSuccess) {
        // Wait for the command to complete (up to 5 seconds)
        VmbBool_t done = VmbBoolFalse;
        for (int i = 0; i < 50 && done == VmbBoolFalse; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            VmbFeatureCommandIsDone(camera_handle_, "GVSPAdjustPacketSize", &done);
        }
        VmbInt64_t packet_size = 0;
        if (VmbFeatureIntGet(camera_handle_, "GVSPPacketSize", &packet_size) == VmbErrorSuccess) {
            RCLCPP_INFO(this->get_logger(), "GVSPAdjustPacketSize: negotiated %ld bytes",
                        packet_size);
        }
    } else if (pkt_err != VmbErrorNotFound && pkt_err != VmbErrorNotSupported) {
        RCLCPP_WARN(this->get_logger(), "GVSPAdjustPacketSize failed: %s",
                    vmb_error_str(pkt_err));
    }
}

void CudaCameraNode::close_camera()
{
    if (!camera_open_) return;

    VmbError_t err = VmbCameraClose(camera_handle_);
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(this->get_logger(), "VmbCameraClose failed: %s (%d)",
                    vmb_error_str(err), static_cast<int>(err));
    }
    camera_handle_ = nullptr;
    camera_open_ = false;

    VmbShutdown();
    RCLCPP_INFO(this->get_logger(), "Camera closed and VmbC shut down");
}

// ============================================================================
// Camera feature configuration
// ============================================================================
void CudaCameraNode::configure_camera_features()
{
    if (!camera_open_) return;

    auto logger = this->get_logger();
    VmbError_t err;

    // ---- Pixel format ----
    std::string pixel_format = this->get_parameter("feature/pixel_format").as_string();
    err = VmbFeatureEnumSet(camera_handle_, "PixelFormat", pixel_format.c_str());
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "Failed to set PixelFormat='%s': %s",
                    pixel_format.c_str(), vmb_error_str(err));
    }

    // ---- Geometry features ----
    VmbInt64_t w = static_cast<VmbInt64_t>(
        this->get_parameter("feature/width").as_int());
    VmbInt64_t h = static_cast<VmbInt64_t>(
        this->get_parameter("feature/height").as_int());
    VmbInt64_t ox = static_cast<VmbInt64_t>(
        this->get_parameter("feature/offset_x").as_int());
    VmbInt64_t oy = static_cast<VmbInt64_t>(
        this->get_parameter("feature/offset_y").as_int());

    // v2.0.0: Override with ROI crop if enabled
    if (roi_enabled_) {
        ox = static_cast<VmbInt64_t>(this->get_parameter("roi_offset_x").as_int());
        oy = static_cast<VmbInt64_t>(this->get_parameter("roi_offset_y").as_int());
        VmbInt64_t roi_w = static_cast<VmbInt64_t>(this->get_parameter("roi_width").as_int());
        VmbInt64_t roi_h = static_cast<VmbInt64_t>(this->get_parameter("roi_height").as_int());
        if (roi_w > 0) w = roi_w;
        if (roi_h > 0) h = roi_h;
        else h = static_cast<VmbInt64_t>(this->get_parameter("feature/height").as_int()) - oy;
        RCLCPP_INFO(logger, "ROI: offset=(%ld,%ld) size=%ldx%ld", ox, oy, w, h);
    }

    // Set offset to 0 first to avoid conflicts when changing size
    VmbFeatureIntSet(camera_handle_, "OffsetX", 0);
    VmbFeatureIntSet(camera_handle_, "OffsetY", 0);

    err = VmbFeatureIntSet(camera_handle_, "Width", w);
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "Failed to set Width=%ld: %s", w, vmb_error_str(err));
    }
    err = VmbFeatureIntSet(camera_handle_, "Height", h);
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "Failed to set Height=%ld: %s", h, vmb_error_str(err));
    }

    VmbFeatureIntSet(camera_handle_, "OffsetX", ox);
    VmbFeatureIntSet(camera_handle_, "OffsetY", oy);

    // ---- Trigger configuration ----
    std::string trigger_selector = this->get_parameter("feature/trigger_selector").as_string();
    std::string trigger_mode = this->get_parameter("feature/trigger_mode").as_string();
    std::string trigger_source = this->get_parameter("feature/trigger_source").as_string();

    VmbFeatureEnumSet(camera_handle_, "TriggerSelector", trigger_selector.c_str());
    VmbFeatureEnumSet(camera_handle_, "TriggerMode", trigger_mode.c_str());
    VmbFeatureEnumSet(camera_handle_, "TriggerSource", trigger_source.c_str());

    // ---- Frame rate ----
    double frame_rate = this->get_parameter("feature/frame_rate").as_double();
    err = VmbFeatureFloatSet(camera_handle_, "AcquisitionFrameRate", frame_rate);
    if (err != VmbErrorSuccess) {
        // Some cameras use AcquisitionFrameRateAbs
        err = VmbFeatureFloatSet(camera_handle_, "AcquisitionFrameRateAbs", frame_rate);
        if (err != VmbErrorSuccess) {
            RCLCPP_WARN(logger, "Failed to set frame rate to %.1f Hz: %s",
                        frame_rate, vmb_error_str(err));
        }
    }

    // ---- Exposure ----
    std::string exposure_auto = this->get_parameter("feature/exposure_auto").as_string();
    err = VmbFeatureEnumSet(camera_handle_, "ExposureAuto", exposure_auto.c_str());
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "Failed to set ExposureAuto='%s': %s",
                    exposure_auto.c_str(), vmb_error_str(err));
    }

    if (exposure_auto == "Off") {
        double exposure_time = this->get_parameter("feature/exposure_time").as_double();
        err = VmbFeatureFloatSet(camera_handle_, "ExposureTime", exposure_time);
        if (err != VmbErrorSuccess) {
            // Try legacy name
            VmbFeatureFloatSet(camera_handle_, "ExposureTimeAbs", exposure_time);
        }
    }

    // ---- Gain ----
    std::string gain_auto = this->get_parameter("feature/gain_auto").as_string();
    err = VmbFeatureEnumSet(camera_handle_, "GainAuto", gain_auto.c_str());
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "Failed to set GainAuto='%s': %s",
                    gain_auto.c_str(), vmb_error_str(err));
    }

    if (gain_auto == "Off") {
        double gain = this->get_parameter("feature/gain").as_double();
        err = VmbFeatureFloatSet(camera_handle_, "Gain", gain);
        if (err != VmbErrorSuccess) {
            VmbFeatureFloatSet(camera_handle_, "GainRaw", gain);
        }
    }

    // ---- White balance ----
    std::string wb_auto = this->get_parameter("feature/balance_white_auto").as_string();
    VmbFeatureEnumSet(camera_handle_, "BalanceWhiteAuto", wb_auto.c_str());

    if (wb_auto == "Off") {
        double ratio_red = this->get_parameter("feature/balance_ratio_red").as_double();
        double ratio_blue = this->get_parameter("feature/balance_ratio_blue").as_double();
        VmbFeatureEnumSet(camera_handle_, "BalanceRatioSelector", "Red");
        VmbFeatureFloatSet(camera_handle_, "BalanceRatio", ratio_red);
        VmbFeatureEnumSet(camera_handle_, "BalanceRatioSelector", "Blue");
        VmbFeatureFloatSet(camera_handle_, "BalanceRatio", ratio_blue);
    }

    // ---- Stream bytes per second (GigE bandwidth) ----
    std::string sbps = this->get_parameter("feature/stream_bytes_per_second").as_string();
    try {
        VmbInt64_t sbps_val = std::stoll(sbps);
        VmbFeatureIntSet(camera_handle_, "StreamBytesPerSecond", sbps_val);
    } catch (...) {
        RCLCPP_WARN(logger, "Invalid stream_bytes_per_second value: %s", sbps.c_str());
    }

    // ---- PTP (Precision Time Protocol) v2.0.0 ----
    // Two PTP params: legacy "use_ptp" (basic enable) and new "enable_ptp_sync"
    // (full IEEE 1588 slave mode with status monitoring).
    if (enable_ptp_sync_ || this->get_parameter("use_ptp").as_bool()) {
        // Try IEEE 1588 PtpMode first (newer Mako firmware)
        err = VmbFeatureEnumSet(camera_handle_, "PtpMode", "Slave");
        if (err == VmbErrorSuccess) {
            RCLCPP_INFO(logger, "PTP: configured as IEEE 1588 Slave");
            // Poll for PTP lock (up to 10 seconds)
            const char* ptp_status = nullptr;
            for (int i = 0; i < 50; ++i) {
                err = VmbFeatureEnumGet(camera_handle_, "PtpStatus", &ptp_status);
                if (err == VmbErrorSuccess && ptp_status) {
                    std::string status(ptp_status);
                    if (status == "Slave" || status == "Master") {
                        RCLCPP_INFO(logger, "PTP: locked (%s) after %d ms", status.c_str(), i * 200);
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            if (!ptp_status || std::string(ptp_status) == "Initializing") {
                RCLCPP_WARN(logger, "PTP: not locked after 10s, timestamps may drift");
            }
        } else {
            // Fall back to legacy PtpEnable boolean
            err = VmbFeatureBoolSet(camera_handle_, "PtpEnable", VmbBoolTrue);
            if (err == VmbErrorSuccess) {
                RCLCPP_INFO(logger, "PTP: enabled (legacy mode)");
            } else {
                RCLCPP_WARN(logger, "PTP: not supported on this camera (%s)", vmb_error_str(err));
            }
        }
        // Enable GevTimestamp latch for frame callback timestamps
        VmbFeatureCommandRun(camera_handle_, "GevTimestampControlLatch");
    }

    // Read back actual width/height from camera
    VmbInt64_t actual_w = 0, actual_h = 0;
    if (VmbFeatureIntGet(camera_handle_, "Width", &actual_w) == VmbErrorSuccess &&
        VmbFeatureIntGet(camera_handle_, "Height", &actual_h) == VmbErrorSuccess) {
        width_ = static_cast<uint32_t>(actual_w);
        height_ = static_cast<uint32_t>(actual_h);
        RCLCPP_INFO(logger, "Camera resolution: %u x %u", width_, height_);
    }

    RCLCPP_INFO(logger, "Camera features configured");
}

// ============================================================================
// Start / stop capture
// ============================================================================
void CudaCameraNode::start_capture()
{
    if (!camera_open_) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start capture: camera not open");
        return;
    }

    auto logger = this->get_logger();
    const size_t bayer_frame_size = static_cast<size_t>(width_) * height_;

    // Ensure buffer pool matches current frame size
    if (buffer_pool_->buffer_size() != bayer_frame_size) {
        RCLCPP_INFO(logger, "Recreating buffer pool for new frame size %zu", bayer_frame_size);
        buffer_pool_ = std::make_unique<PinnedBufferPool>(
            static_cast<size_t>(num_buffers_), bayer_frame_size);
    }

    // Prepare VmbFrame_t structures and announce buffers
    vmb_frames_.resize(static_cast<size_t>(num_buffers_));
    for (int i = 0; i < num_buffers_; ++i) {
        std::memset(&vmb_frames_[i], 0, sizeof(VmbFrame_t));
        vmb_frames_[i].buffer = buffer_pool_->get_buffer(static_cast<size_t>(i));
        vmb_frames_[i].bufferSize = static_cast<VmbUint32_t>(bayer_frame_size);
        vmb_frames_[i].context[0] = this;  // Pass node pointer for static callback
        vmb_frames_[i].context[1] = reinterpret_cast<void*>(static_cast<uintptr_t>(i));

        VmbError_t err = VmbFrameAnnounce(camera_handle_, &vmb_frames_[i],
                                          sizeof(VmbFrame_t));
        if (err != VmbErrorSuccess) {
            RCLCPP_ERROR(logger, "VmbFrameAnnounce(%d) failed: %s",
                         i, vmb_error_str(err));
            return;
        }
    }

    // Start capture engine
    VMB_CHECK(VmbCaptureStart(camera_handle_), logger, "VmbCaptureStart failed");

    // Queue all frames with callback
    for (int i = 0; i < num_buffers_; ++i) {
        VmbError_t err = VmbCaptureFrameQueue(camera_handle_, &vmb_frames_[i],
                                              &CudaCameraNode::frame_callback);
        if (err != VmbErrorSuccess) {
            RCLCPP_ERROR(logger, "VmbCaptureFrameQueue(%d) failed: %s",
                         i, vmb_error_str(err));
        }
    }

    // Start acquisition on camera
    VMB_CHECK(VmbFeatureCommandRun(camera_handle_, "AcquisitionStart"),
              logger, "AcquisitionStart failed");

    // Start publisher thread
    running_.store(true, std::memory_order_release);
    pub_thread_ = std::thread(&CudaCameraNode::publisher_loop, this);

    RCLCPP_INFO(logger, "Capture started with %d pinned buffers", num_buffers_);
}

void CudaCameraNode::stop_capture()
{
    if (!camera_open_) return;

    auto logger = this->get_logger();

    // Stop publisher thread first
    running_.store(false, std::memory_order_release);
    if (pub_thread_.joinable()) {
        pub_thread_.join();
    }

    // Stop acquisition
    VmbError_t err = VmbFeatureCommandRun(camera_handle_, "AcquisitionStop");
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "AcquisitionStop failed: %s", vmb_error_str(err));
    }

    // End capture
    err = VmbCaptureEnd(camera_handle_);
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "VmbCaptureEnd failed: %s", vmb_error_str(err));
    }

    // Flush and revoke frames
    VmbCaptureQueueFlush(camera_handle_);
    VmbFrameRevokeAll(camera_handle_);
    vmb_frames_.clear();

    RCLCPP_INFO(logger, "Capture stopped");
}

// ============================================================================
// Frame callback (static C function, runs on Vimba internal thread)
//
// FIX D1: Do NOT re-queue the frame here. The publisher thread will re-queue
//         after it has finished reading the pinned buffer, preventing the
//         buffer aliasing race where Vimba could overwrite the buffer while
//         the publisher is still reading it.
//
// FIX D6: Read cached atomic use_ptp_ / ptp_offset_ instead of calling
//         get_parameter() on the Vimba callback thread.
//
// FIX D2: Use system_clock (wall clock) instead of steady_clock (monotonic).
// ============================================================================
void VMB_CALL CudaCameraNode::frame_callback(
    VmbHandle_t /*camera_handle*/,
    VmbHandle_t /*stream_handle*/,
    VmbFrame_t* frame)
{
    if (!frame) return;

    auto* node = static_cast<CudaCameraNode*>(frame->context[0]);
    if (!node) return;

    size_t vmb_frame_index = reinterpret_cast<uintptr_t>(frame->context[1]);

    // Only process complete frames
    if (frame->receiveStatus != VmbFrameStatusComplete) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Frame incomplete, status=%d", frame->receiveStatus);
        // Re-queue incomplete frames immediately (buffer not used by publisher)
        VmbCaptureFrameQueue(node->camera_handle_, frame,
                             &CudaCameraNode::frame_callback);
        return;
    }

    // Determine timestamp (D6: use cached atomics, D2: use system_clock)
    uint64_t timestamp_ns;
    if (node->use_ptp_.load(std::memory_order_relaxed)) {
        // Use camera PTP timestamp with offset
        timestamp_ns = frame->timestamp;
        int64_t offset = node->ptp_offset_.load(std::memory_order_relaxed);
        timestamp_ns = static_cast<uint64_t>(
            static_cast<int64_t>(timestamp_ns) + offset);
    } else {
        // Use wall clock (system_clock) for ROS compatibility (fix D2)
        auto now = std::chrono::system_clock::now();
        timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count());
    }

    // Build queue entry
    size_t buffer_index = reinterpret_cast<uintptr_t>(frame->context[1]);
    FrameEntry entry;
    entry.buffer_index = buffer_index;
    entry.vmb_frame_index = vmb_frame_index;
    entry.frame_id = frame->frameID;
    entry.timestamp_ns = timestamp_ns;
    entry.width = frame->width;
    entry.height = frame->height;

    // GAP 3: Update watchdog timestamp
    {
        auto wall_now = std::chrono::system_clock::now();
        node->last_frame_time_ns_.store(
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                wall_now.time_since_epoch()).count()),
            std::memory_order_relaxed);
    }

    // Push to lock-free queue (no mutex, no allocation)
    // FIX D1: Do NOT re-queue the frame here. If the queue is full, we must
    // still re-queue since the publisher won't see this frame.
    if (!node->frame_queue_->try_push(std::move(entry))) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "SPSC queue full, dropping frame %lu", frame->frameID);
        // Queue was full, publisher won't process this frame, so re-queue now
        VmbCaptureFrameQueue(node->camera_handle_, frame,
                             &CudaCameraNode::frame_callback);
    }
    // If push succeeded: publisher thread will re-queue after it finishes
    // reading the buffer (fix D1).
}

// ============================================================================
// Publisher thread
//
// FIX D1: After finishing with the pinned buffer (debayer + memcpy done),
//         re-queue the VmbFrame to Vimba. This ensures Vimba never overwrites
//         a buffer that is still being read.
// ============================================================================
void CudaCameraNode::publisher_loop()
{
    RCLCPP_INFO(this->get_logger(), "Publisher thread started");

    std::string frame_id = this->get_parameter("frame_id").as_string();

    // Allocate pinned output buffer for RGB (host-side, for zero-copy readback)
    const size_t rgb_size = static_cast<size_t>(width_) * height_ * 3;
    uint8_t* h_rgb_output = nullptr;
    cudaError_t cuda_err = cudaMallocHost(&h_rgb_output, rgb_size);
    if (cuda_err != cudaSuccess) {
        RCLCPP_ERROR(this->get_logger(), "cudaMallocHost(rgb_output) failed: %s",
                     cudaGetErrorString(cuda_err));
        return;
    }

    FrameEntry entry;
    cudaStream_t stream = static_cast<cudaStream_t>(cuda_stream_);

    while (running_.load(std::memory_order_acquire)) {
        // Try to pop from the SPSC queue
        if (!frame_queue_->try_pop(entry)) {
            // No frame available; yield briefly
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        // Get pointer to the pinned Bayer buffer
        const uint8_t* bayer_data = buffer_pool_->get_buffer(entry.buffer_index);
        const size_t bayer_size = static_cast<size_t>(entry.width) * entry.height;

        if (publish_raw_bayer_) {
            // ================================================================
            // RAW BAYER PATH: zero GPU work, minimum latency (~0.1ms)
            // Publish BayerRG8 directly from pinned buffer.
            // Perception ISP handles debayer+undistort+resize in one JAX kernel.
            // ================================================================

            // GPU-direct: expose pinned Bayer pointer directly (no GPU copy needed —
            // pinned memory is GPU-accessible via unified addressing)
            if (gpu_direct_enabled_) {
                std::lock_guard<std::mutex> lock(gpu_frame_mutex_);
                GpuFrame gf;
                gf.device_ptr = const_cast<uint8_t*>(bayer_data);  // pinned = GPU-accessible
                gf.width = static_cast<int>(entry.width);
                gf.height = static_cast<int>(entry.height);
                gf.channels = 1;
                gf.encoding = "bayer_rggb8";
                gf.timestamp_ns = entry.timestamp_ns;
                gf.buffer_index = static_cast<int>(entry.buffer_index);
                gf.frame_id = entry.frame_id;
                latest_gpu_frame_ = gf;
            }

            // Build ROS Image message — copy Bayer directly (1 byte/pixel)
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp.sec = static_cast<int32_t>(entry.timestamp_ns / 1000000000ULL);
            img_msg->header.stamp.nanosec = static_cast<uint32_t>(entry.timestamp_ns % 1000000000ULL);
            img_msg->header.frame_id = frame_id;
            img_msg->width = entry.width;
            img_msg->height = entry.height;
            img_msg->encoding = "bayer_rggb8";
            img_msg->is_bigendian = false;
            img_msg->step = entry.width * 1;  // 1 byte/pixel
            img_msg->data.resize(bayer_size);
            std::memcpy(img_msg->data.data(), bayer_data, bayer_size);

            // Re-queue AFTER memcpy (D1 fix)
            if (entry.vmb_frame_index < vmb_frames_.size()) {
                VmbCaptureFrameQueue(camera_handle_,
                                     &vmb_frames_[entry.vmb_frame_index],
                                     &CudaCameraNode::frame_callback);
            }

            // Publish
            auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(
                camera_info_mgr_->getCameraInfo());
            info_msg->header = img_msg->header;
            camera_pub_.publish(*img_msg, *info_msg);

        } else {
            // ================================================================
            // RGB PATH: CUDA debayer, compatible output for rviz/debugging
            // ================================================================

            cuda_debayer(
                bayer_data,
                d_rgb_output_,
                static_cast<int>(entry.width),
                static_cast<int>(entry.height),
                stream);

            // GPU-direct: copy debayered RGB into ring buffer
            if (gpu_direct_enabled_) {
                int wr = gpu_direct_write_idx_.load(std::memory_order_relaxed);
                for (int tries = 0; tries < kMaxGpuDirectBuffers; ++tries) {
                    int candidate = (wr + tries) % kMaxGpuDirectBuffers;
                    if (!gpu_direct_slot_held_[candidate].load(std::memory_order_acquire)) {
                        wr = candidate;
                        break;
                    }
                }
                cudaMemcpyAsync(d_gpu_direct_bufs_[wr], d_rgb_output_, rgb_size,
                                cudaMemcpyDeviceToDevice, stream);
                cudaStreamSynchronize(stream);
                {
                    std::lock_guard<std::mutex> lock(gpu_frame_mutex_);
                    GpuFrame gf;
                    gf.device_ptr = d_gpu_direct_bufs_[wr];
                    gf.width = static_cast<int>(entry.width);
                    gf.height = static_cast<int>(entry.height);
                    gf.channels = 3;
                    gf.encoding = "rgb8";
                    gf.timestamp_ns = entry.timestamp_ns;
                    gf.buffer_index = wr;
                    gf.frame_id = entry.frame_id;
                    latest_gpu_frame_ = gf;
                }
                gpu_direct_write_idx_.store(
                    (wr + 1) % kMaxGpuDirectBuffers, std::memory_order_relaxed);
            }

            // Copy RGB to host
            cudaMemcpyAsync(h_rgb_output, d_rgb_output_, rgb_size,
                            cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);

            // Re-queue AFTER all reads done (D1 fix)
            if (entry.vmb_frame_index < vmb_frames_.size()) {
                VmbCaptureFrameQueue(camera_handle_,
                                     &vmb_frames_[entry.vmb_frame_index],
                                     &CudaCameraNode::frame_callback);
            }

            // Build RGB ROS Image message
            auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
            img_msg->header.stamp.sec = static_cast<int32_t>(entry.timestamp_ns / 1000000000ULL);
            img_msg->header.stamp.nanosec = static_cast<uint32_t>(entry.timestamp_ns % 1000000000ULL);
            img_msg->header.frame_id = frame_id;
            img_msg->width = entry.width;
            img_msg->height = entry.height;
            img_msg->encoding = "rgb8";
            img_msg->is_bigendian = false;
            img_msg->step = entry.width * 3;
            const size_t frame_rgb_size = static_cast<size_t>(entry.width) * entry.height * 3;
            img_msg->data.resize(frame_rgb_size);
            std::memcpy(img_msg->data.data(), h_rgb_output, frame_rgb_size);

            auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(
                camera_info_mgr_->getCameraInfo());
            info_msg->header = img_msg->header;
            camera_pub_.publish(*img_msg, *info_msg);
        }

        // ---- Build CameraInfo message ----
        auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(
            camera_info_mgr_->getCameraInfo());
        info_msg->header = img_msg->header;

        // ---- Publish via image_transport (fix A) ----
        camera_pub_.publish(*img_msg, *info_msg);
    }

    // Cleanup pinned output buffer
    if (h_rgb_output) {
        cudaFreeHost(h_rgb_output);
    }

    RCLCPP_INFO(this->get_logger(), "Publisher thread exiting");
}

// ============================================================================
// Service callbacks
// ============================================================================
void CudaCameraNode::on_start_stream(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (running_.load()) {
        response->success = false;
        response->message = "Capture already running";
        return;
    }

    if (!camera_open_) {
        open_camera();
        if (camera_open_) {
            configure_camera_features();
        }
    }

    if (camera_open_) {
        start_capture();
        response->success = true;
        response->message = "Capture started";
    } else {
        response->success = false;
        response->message = "Failed to open camera";
    }
}

void CudaCameraNode::on_stop_stream(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!running_.load()) {
        response->success = false;
        response->message = "Capture not running";
        return;
    }

    stop_capture();
    response->success = true;
    response->message = "Capture stopped";
}

void CudaCameraNode::on_load_settings(
    const std::shared_ptr<avt_vimba_camera_msgs::srv::LoadSettings::Request> request,
    std::shared_ptr<avt_vimba_camera_msgs::srv::LoadSettings::Response> response)
{
    RCLCPP_WARN(this->get_logger(),
                "load_settings service called with path='%s' but is not yet implemented "
                "in the CUDA camera node", request->input_path.c_str());
    response->result = false;
    response->message = "load_settings not yet implemented in CUDA camera node";
}

void CudaCameraNode::on_save_settings(
    const std::shared_ptr<avt_vimba_camera_msgs::srv::SaveSettings::Request> request,
    std::shared_ptr<avt_vimba_camera_msgs::srv::SaveSettings::Response> response)
{
    RCLCPP_WARN(this->get_logger(),
                "save_settings service called with path='%s' but is not yet implemented "
                "in the CUDA camera node", request->output_path.c_str());
    response->result = false;
    response->message = "save_settings not yet implemented in CUDA camera node";
}

// ============================================================================
// Dynamic parameter callback
// ============================================================================
rcl_interfaces::msg::SetParametersResult CudaCameraNode::on_parameter_change(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    bool geometry_changed = false;

    for (const auto& param : parameters) {
        const std::string& name = param.get_name();

        // Update cached PTP parameters for callback thread (D6 fix)
        if (name == "use_ptp") {
            use_ptp_.store(param.as_bool(), std::memory_order_relaxed);
            continue;
        }
        if (name == "ptp_offset") {
            ptp_offset_.store(param.as_int(), std::memory_order_relaxed);
            continue;
        }

        // Only handle feature/* parameters dynamically
        if (name.rfind("feature/", 0) != 0) {
            continue;
        }

        // Extract the feature name (strip "feature/" prefix)
        std::string feature_name = name.substr(8);

        // Check if this is a geometry feature that requires restart
        if (feature_name == "width" || feature_name == "height" ||
            feature_name == "offset_x" || feature_name == "offset_y" ||
            feature_name == "pixel_format") {
            geometry_changed = true;
            continue;  // Will be applied after stop/start
        }

        if (!camera_open_ || enable_pcap_) continue;

        // Check writability for dynamically discovered features
        auto disc_it = discovered_features_.find(feature_name);
        if (disc_it != discovered_features_.end() && !disc_it->second) {
            RCLCPP_WARN(this->get_logger(),
                        "Feature '%s' is read-only, ignoring parameter change",
                        feature_name.c_str());
            continue;
        }

        // Map parameter names to VmbC feature names
        std::string vmb_feature;
        if (feature_name == "frame_rate") vmb_feature = "AcquisitionFrameRate";
        else if (feature_name == "exposure_auto") vmb_feature = "ExposureAuto";
        else if (feature_name == "exposure_time") vmb_feature = "ExposureTime";
        else if (feature_name == "gain_auto") vmb_feature = "GainAuto";
        else if (feature_name == "gain") vmb_feature = "Gain";
        else if (feature_name == "balance_white_auto") vmb_feature = "BalanceWhiteAuto";
        else if (feature_name == "trigger_source") vmb_feature = "TriggerSource";
        else if (feature_name == "trigger_mode") vmb_feature = "TriggerMode";
        else if (feature_name == "trigger_selector") vmb_feature = "TriggerSelector";
        else vmb_feature = feature_name;  // Pass-through

        VmbError_t err = VmbErrorSuccess;

        switch (param.get_type()) {
            case rclcpp::ParameterType::PARAMETER_STRING:
                err = VmbFeatureEnumSet(camera_handle_, vmb_feature.c_str(),
                                        param.as_string().c_str());
                if (err != VmbErrorSuccess) {
                    // Try as command
                    err = VmbFeatureCommandRun(camera_handle_, vmb_feature.c_str());
                }
                break;
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
                err = VmbFeatureFloatSet(camera_handle_, vmb_feature.c_str(),
                                         param.as_double());
                break;
            case rclcpp::ParameterType::PARAMETER_INTEGER:
                err = VmbFeatureIntSet(camera_handle_, vmb_feature.c_str(),
                                       param.as_int());
                break;
            case rclcpp::ParameterType::PARAMETER_BOOL:
                err = VmbFeatureBoolSet(camera_handle_, vmb_feature.c_str(),
                                        param.as_bool() ? VmbBoolTrue : VmbBoolFalse);
                break;
            default:
                RCLCPP_WARN(this->get_logger(),
                            "Unsupported parameter type for '%s'", name.c_str());
                break;
        }

        if (err != VmbErrorSuccess) {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set feature '%s': %s",
                        vmb_feature.c_str(), vmb_error_str(err));
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Feature '%s' updated via parameter '%s'",
                        vmb_feature.c_str(), name.c_str());
        }
    }

    // If geometry changed, stop/reconfigure/restart
    if (geometry_changed && camera_open_) {
        RCLCPP_INFO(this->get_logger(),
                    "Geometry parameters changed, restarting capture...");
        bool was_running = running_.load();
        if (was_running) {
            stop_capture();
        }

        // Update member variables
        width_ = static_cast<uint32_t>(this->get_parameter("feature/width").as_int());
        height_ = static_cast<uint32_t>(this->get_parameter("feature/height").as_int());

        // Reallocate GPU buffers
        if (d_rgb_output_) cudaFree(d_rgb_output_);

        const size_t rgb_size = static_cast<size_t>(width_) * height_ * 3;
        cudaMalloc(&d_rgb_output_, rgb_size);

        // Recreate buffer pool for new frame size
        const size_t bayer_size = static_cast<size_t>(width_) * height_;
        buffer_pool_ = std::make_unique<PinnedBufferPool>(
            static_cast<size_t>(num_buffers_), bayer_size);

        configure_camera_features();

        if (was_running) {
            start_capture();
        }
    }

    return result;
}

// ============================================================================
// GAP 1: PCAP Replay Mode
// ============================================================================
void CudaCameraNode::start_pcap_replay()
{
    if (!pcap_reader_ || !pcap_reader_->isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot start PCAP replay: reader not initialized");
        return;
    }

    setup_terminal();
    print_keyboard_controls();

    // Start publisher thread (reused for both live and PCAP paths)
    running_.store(true, std::memory_order_release);
    pub_thread_ = std::thread(&CudaCameraNode::publisher_loop, this);

    // Start keyboard input thread
    keyboard_thread_running_.store(true);
    keyboard_thread_ = std::thread(&CudaCameraNode::keyboard_input_thread_func, this);

    // Start PCAP replay thread (reads frames -> pinned buffer -> SPSC queue)
    pcap_thread_running_.store(true);
    pcap_thread_ = std::thread(&CudaCameraNode::pcap_replay_thread_func, this);
}

void CudaCameraNode::pcap_replay_thread_func()
{
    const auto base_frame_interval = std::chrono::milliseconds(33);
    const int frames_per_seek = static_cast<int>(pcap_seek_time_ * pcap_assumed_fps_);

    while (pcap_thread_running_.load() && rclcpp::ok()) {
        // Handle seek forward
        if (pcap_step_forward_.load()) {
            pcap_step_forward_.store(false);
            GigEFrame gige_frame;
            bool success = false;

            for (int i = 0; i < frames_per_seek; ++i) {
                if (pcap_reader_->readNextFrame(gige_frame)) {
                    pcap_frame_index_++;
                    success = true;
                } else {
                    RCLCPP_INFO(this->get_logger(), "[SEEK] End of PCAP");
                    break;
                }
            }

            if (success) {
                // Copy Bayer data to pinned buffer and push to SPSC queue
                size_t buf_idx = static_cast<size_t>(pcap_frame_index_.load() % num_buffers_);
                size_t bayer_size = static_cast<size_t>(width_) * height_;
                size_t copy_size = std::min(gige_frame.data.size(), bayer_size);
                uint8_t* pinned = buffer_pool_->get_buffer(buf_idx);
                std::memcpy(pinned, gige_frame.data.data(), copy_size);

                FrameEntry entry;
                entry.buffer_index = buf_idx;
                entry.vmb_frame_index = SIZE_MAX;  // No VmbFrame to re-queue
                entry.frame_id = gige_frame.frame_id;
                auto now = std::chrono::system_clock::now();
                entry.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch()).count());
                entry.width = width_;
                entry.height = height_;

                frame_queue_->try_push(std::move(entry));
                RCLCPP_INFO(this->get_logger(), "[SEEK] +%.1fs (frame %d)",
                            pcap_seek_time_, pcap_frame_index_.load() - 1);
            }
        }
        // Handle seek backward
        else if (pcap_step_backward_.load()) {
            pcap_step_backward_.store(false);
            int target_frame = std::max(0, pcap_frame_index_.load() - frames_per_seek - 1);

            GigEFrame gige_frame;
            if (pcap_reader_->seekToFrame(target_frame) &&
                pcap_reader_->readNextFrame(gige_frame)) {
                pcap_frame_index_.store(target_frame + 1);

                size_t buf_idx = static_cast<size_t>(pcap_frame_index_.load() % num_buffers_);
                size_t bayer_size = static_cast<size_t>(width_) * height_;
                size_t copy_size = std::min(gige_frame.data.size(), bayer_size);
                uint8_t* pinned = buffer_pool_->get_buffer(buf_idx);
                std::memcpy(pinned, gige_frame.data.data(), copy_size);

                FrameEntry entry;
                entry.buffer_index = buf_idx;
                entry.vmb_frame_index = SIZE_MAX;
                entry.frame_id = gige_frame.frame_id;
                auto now = std::chrono::system_clock::now();
                entry.timestamp_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        now.time_since_epoch()).count());
                entry.width = width_;
                entry.height = height_;

                frame_queue_->try_push(std::move(entry));
                RCLCPP_INFO(this->get_logger(), "[SEEK] -%.1fs (frame %d)",
                            pcap_seek_time_, target_frame);
            } else {
                RCLCPP_WARN(this->get_logger(), "[SEEK] Cannot seek backward");
            }
        }

        // Handle pause
        if (pcap_paused_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto start_time = std::chrono::steady_clock::now();
        GigEFrame gige_frame;

        if (pcap_reader_->readNextFrame(gige_frame)) {
            pcap_frame_index_++;

            // Copy Bayer data to pinned buffer and push to SPSC queue
            size_t buf_idx = static_cast<size_t>(pcap_frame_index_.load() % num_buffers_);
            size_t bayer_size = static_cast<size_t>(width_) * height_;
            size_t copy_size = std::min(gige_frame.data.size(), bayer_size);
            uint8_t* pinned = buffer_pool_->get_buffer(buf_idx);
            std::memcpy(pinned, gige_frame.data.data(), copy_size);

            FrameEntry entry;
            entry.buffer_index = buf_idx;
            entry.vmb_frame_index = SIZE_MAX;  // No VmbFrame to re-queue
            entry.frame_id = gige_frame.frame_id;
            auto now = std::chrono::system_clock::now();
            entry.timestamp_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    now.time_since_epoch()).count());
            entry.width = width_;
            entry.height = height_;

            if (!frame_queue_->try_push(std::move(entry))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "SPSC queue full during PCAP replay, dropping frame");
            }

            // Playback speed control
            double speed = pcap_playback_speed_.load();
            auto adjusted_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                base_frame_interval * (1.0 / speed));
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time);

            if (elapsed < adjusted_interval) {
                std::this_thread::sleep_for(adjusted_interval - elapsed);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "PCAP replay completed (%d frames)",
                        pcap_frame_index_.load());
            pcap_thread_running_.store(false);
            break;
        }
    }
}

void CudaCameraNode::setup_terminal()
{
    tty_fd_ = ::open("/dev/tty", O_RDWR | O_NONBLOCK);
    if (tty_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Cannot open /dev/tty - keyboard controls disabled");
        return;
    }

    tcgetattr(tty_fd_, &orig_termios_);
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    tcsetattr(tty_fd_, TCSANOW, &raw);
}

void CudaCameraNode::restore_terminal()
{
    if (tty_fd_ >= 0) {
        tcsetattr(tty_fd_, TCSANOW, &orig_termios_);
        ::close(tty_fd_);
        tty_fd_ = -1;
    }
}

void CudaCameraNode::print_keyboard_controls()
{
    auto logger = this->get_logger();
    RCLCPP_INFO(logger, "");
    RCLCPP_INFO(logger, "=======================================================");
    RCLCPP_INFO(logger, "  PCAP Playback Controls:");
    RCLCPP_INFO(logger, "  SPACE     - Pause/Resume");
    RCLCPP_INFO(logger, "  UP/DOWN   - Speed %.1fx-%.1fx (%.1fx increments)",
                pcap_speed_min_, pcap_speed_max_, pcap_speed_increment_);
    RCLCPP_INFO(logger, "  LEFT      - Seek backward %.1fs", pcap_seek_time_);
    RCLCPP_INFO(logger, "  RIGHT     - Seek forward %.1fs", pcap_seek_time_);
    RCLCPP_INFO(logger, "  r         - Reset speed to 1.0x");
    RCLCPP_INFO(logger, "  q         - Quit");
    RCLCPP_INFO(logger, "=======================================================");
    RCLCPP_INFO(logger, "");
}

void CudaCameraNode::keyboard_input_thread_func()
{
    if (tty_fd_ < 0) return;

    while (keyboard_thread_running_.load() && rclcpp::ok()) {
        char c;
        if (read(tty_fd_, &c, 1) == 1) {
            if (c == ' ') {
                bool was_paused = pcap_paused_.load();
                pcap_paused_.store(!was_paused);
                RCLCPP_INFO(this->get_logger(), was_paused ? "[RESUME]" : "[PAUSE]");
            } else if (c == 27) {
                char seq[2];
                if (read(tty_fd_, &seq[0], 1) == 1 &&
                    read(tty_fd_, &seq[1], 1) == 1 && seq[0] == '[') {
                    if (seq[1] == 'A') {  // UP
                        double new_speed = std::min(
                            pcap_playback_speed_.load() + pcap_speed_increment_,
                            pcap_speed_max_);
                        pcap_playback_speed_.store(new_speed);
                        RCLCPP_INFO(this->get_logger(), "[SPEED] %.1fx", new_speed);
                    } else if (seq[1] == 'B') {  // DOWN
                        double new_speed = std::max(
                            pcap_playback_speed_.load() - pcap_speed_increment_,
                            pcap_speed_min_);
                        pcap_playback_speed_.store(new_speed);
                        RCLCPP_INFO(this->get_logger(), "[SPEED] %.1fx", new_speed);
                    } else if (seq[1] == 'D') {  // LEFT
                        pcap_step_backward_.store(true);
                    } else if (seq[1] == 'C') {  // RIGHT
                        pcap_step_forward_.store(true);
                    }
                }
            } else if (c == 'r' || c == 'R') {
                pcap_playback_speed_.store(1.0);
                RCLCPP_INFO(this->get_logger(), "[SPEED] Reset to 1.0x");
            } else if (c == 'q' || c == 'Q') {
                RCLCPP_INFO(this->get_logger(), "[QUIT]");
                pcap_thread_running_.store(false);
                keyboard_thread_running_.store(false);
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ============================================================================
// GAP 2: Dynamic Feature Discovery
// ============================================================================
void CudaCameraNode::discover_camera_features()
{
    if (!camera_open_) return;

    auto logger = this->get_logger();

    // Query number of features
    VmbUint32_t feature_count = 0;
    VmbError_t err = VmbFeaturesList(camera_handle_, nullptr, 0, &feature_count, 0);
    if (err != VmbErrorSuccess || feature_count == 0) {
        RCLCPP_WARN(logger, "VmbFeaturesList count query failed or returned 0 features");
        return;
    }

    // Allocate and fetch feature info
    std::vector<VmbFeatureInfo_t> features(feature_count);
    VmbUint32_t features_filled = 0;
    err = VmbFeaturesList(camera_handle_, features.data(), feature_count,
                          &features_filled, sizeof(VmbFeatureInfo_t));
    if (err != VmbErrorSuccess) {
        RCLCPP_WARN(logger, "VmbFeaturesList failed: %s", vmb_error_str(err));
        return;
    }

    uint32_t declared_count = 0;

    for (VmbUint32_t i = 0; i < features_filled; ++i) {
        const VmbFeatureInfo_t& fi = features[i];
        if (!fi.name) continue;

        std::string feature_name(fi.name);
        std::string param_name = "feature/" + feature_name;

        // Skip features we already declared as hardcoded parameters
        if (this->has_parameter(param_name)) {
            // Track writability for the parameter change handler
            VmbBool_t writable = VmbBoolFalse;
            VmbFeatureAccessQuery(camera_handle_, fi.name, nullptr, &writable);
            discovered_features_[feature_name] = (writable == VmbBoolTrue);
            continue;
        }

        // Check access
        VmbBool_t readable = VmbBoolFalse;
        VmbBool_t writable = VmbBoolFalse;
        VmbFeatureAccessQuery(camera_handle_, fi.name, &readable, &writable);

        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = (writable != VmbBoolTrue);

        // Build description
        std::string desc_str;
        if (fi.category) desc_str += std::string("[") + fi.category + "] ";
        if (fi.description) desc_str += fi.description;
        if (fi.unit) desc_str += std::string(" (Unit: ") + fi.unit + ")";
        descriptor.description = desc_str;

        bool declared = false;

        switch (fi.featureDataType) {
            case VmbFeatureDataBool: {
                VmbBool_t val = VmbBoolFalse;
                VmbFeatureBoolGet(camera_handle_, fi.name, &val);
                try {
                    this->declare_parameter<bool>(param_name, val == VmbBoolTrue, descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            case VmbFeatureDataInt: {
                VmbInt64_t val = 0;
                VmbFeatureIntGet(camera_handle_, fi.name, &val);
                try {
                    this->declare_parameter<int64_t>(param_name, val, descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            case VmbFeatureDataFloat: {
                double val = 0.0;
                VmbFeatureFloatGet(camera_handle_, fi.name, &val);
                try {
                    this->declare_parameter<double>(param_name, val, descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            case VmbFeatureDataString: {
                VmbUint32_t str_len = 0;
                VmbFeatureStringGet(camera_handle_, fi.name, nullptr, 0, &str_len);
                std::string val(str_len + 1, '\0');
                VmbFeatureStringGet(camera_handle_, fi.name, val.data(),
                                    static_cast<VmbUint32_t>(val.size()), &str_len);
                val.resize(str_len > 0 ? str_len - 1 : 0);  // strip null
                try {
                    this->declare_parameter<std::string>(param_name, val, descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            case VmbFeatureDataEnum: {
                const char* val = nullptr;
                VmbFeatureEnumGet(camera_handle_, fi.name, &val);
                std::string val_str = val ? val : "";
                try {
                    this->declare_parameter<std::string>(param_name, val_str, descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            case VmbFeatureDataCommand: {
                // Commands are exposed as string params; writing "Run" executes them
                try {
                    this->declare_parameter<std::string>(param_name, "", descriptor);
                    declared = true;
                } catch (...) {}
                break;
            }
            default:
                break;
        }

        if (declared) {
            discovered_features_[feature_name] = (writable == VmbBoolTrue);
            declared_count++;
        }
    }

    RCLCPP_INFO(logger, "Dynamic feature discovery: %u features declared as ROS parameters "
                "(%zu total tracked)", declared_count, discovered_features_.size());
}

// ============================================================================
// GAP 3: Watchdog / Reconnect
// ============================================================================
void CudaCameraNode::watchdog_callback()
{
    if (!camera_open_ || !running_.load() || enable_pcap_) return;

    uint64_t last_ns = last_frame_time_ns_.load(std::memory_order_relaxed);
    if (last_ns == 0) return;  // No frame received yet

    auto now = std::chrono::system_clock::now();
    uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count());

    double elapsed_sec = static_cast<double>(now_ns - last_ns) / 1e9;

    if (elapsed_sec > watchdog_timeout_sec_) {
        RCLCPP_WARN(this->get_logger(),
                     "Watchdog: no frame received for %.1f seconds, attempting reconnect...",
                     elapsed_sec);
        attempt_reconnect();
    }
}

void CudaCameraNode::attempt_reconnect()
{
    auto logger = this->get_logger();

    // Stop current capture
    RCLCPP_INFO(logger, "Watchdog reconnect: stopping capture...");
    stop_capture();

    // Close camera
    RCLCPP_INFO(logger, "Watchdog reconnect: closing camera...");
    close_camera();

    // Re-open camera (with retry loop inside open_camera())
    RCLCPP_INFO(logger, "Watchdog reconnect: re-opening camera...");
    open_camera();

    if (!camera_open_) {
        RCLCPP_ERROR(logger, "Watchdog reconnect: failed to re-open camera");
        return;
    }

    // Re-configure features
    RCLCPP_INFO(logger, "Watchdog reconnect: re-configuring features...");
    configure_camera_features();

    // Reset watchdog timestamp so we don't immediately trigger again
    last_frame_time_ns_.store(0, std::memory_order_relaxed);

    // Restart capture
    RCLCPP_INFO(logger, "Watchdog reconnect: restarting capture...");
    start_capture();

    RCLCPP_INFO(logger, "Watchdog reconnect: complete");
}

}  // namespace avt_vimba_camera

// ============================================================================
// Component registration (fix I3: no main() here; cuda_camera_exec.cpp has it)
// ============================================================================
RCLCPP_COMPONENTS_REGISTER_NODE(avt_vimba_camera::CudaCameraNode)
