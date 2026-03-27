#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <map>
#include <termios.h>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <avt_vimba_camera_msgs/srv/load_settings.hpp>
#include <avt_vimba_camera_msgs/srv/save_settings.hpp>

#include "avt_vimba_camera/pinned_buffer_pool.hpp"
#include "avt_vimba_camera/spsc_queue.hpp"
#include "avt_vimba_camera/pcap_reader.hpp"

// Forward-declare VmbC types to avoid pulling in the full header here.
// VMB_CALL is the Vimba calling convention macro (empty on Linux).
#ifndef VMB_CALL
#define VMB_CALL
#endif

struct VmbFrame;
typedef struct VmbFrame VmbFrame_t;
typedef void* VmbHandle_t;

namespace avt_vimba_camera {

/**
 * Metadata carried through the SPSC queue from the Vimba callback
 * to the publisher thread.
 */
struct FrameEntry {
    size_t buffer_index;        ///< Index into PinnedBufferPool
    size_t vmb_frame_index;     ///< Index into vmb_frames_ for re-queuing
    uint64_t frame_id;          ///< Vimba frame ID
    uint64_t timestamp_ns;      ///< Timestamp in nanoseconds
    uint32_t width;
    uint32_t height;
};

/**
 * ROS2 camera node with CUDA-accelerated debayering.
 *
 * Alternative to the stock avt_vimba_camera mono_camera_node with:
 * - Pinned memory DMA (zero-copy from camera to GPU)
 * - Malvar-He-Cutler GPU demosaicing
 * - uint8 RGB Image msg output via image_transport
 *
 * Published topics (via image_transport::CameraPublisher):
 *   ~/image    (sensor_msgs/Image + CameraInfo)
 *
 * Services:
 *   ~/start_stream   (std_srvs/Trigger)
 *   ~/stop_stream    (std_srvs/Trigger)
 *   ~/load_settings  (avt_vimba_camera_msgs/LoadSettings)  [stub]
 *   ~/save_settings  (avt_vimba_camera_msgs/SaveSettings)  [stub]
 *
 * Parameters:
 *   camera_id            (string)  - Vimba camera ID or IP
 *   camera_info_url      (string)  - camera_info_manager calibration URL
 *   frame_rate           (double)  - acquisition frame rate (Hz)
 *   exposure_auto        (string)  - "Off", "Once", "Continuous"
 *   exposure_time        (double)  - manual exposure time (us)
 *   gain_auto            (string)  - "Off", "Once", "Continuous"
 *   gain                 (double)  - manual gain (dB)
 *   num_buffers          (int)     - number of pinned DMA buffers
 *   use_ptp              (bool)    - use PTP timestamps from camera
 *   ptp_offset           (int)     - nanosecond offset to add to PTP timestamps
 */
/**
 * GPU frame descriptor for zero-copy IPC.
 *
 * When the camera driver and perception node are composed into the same
 * process, downstream consumers can call get_latest_gpu_frame() to obtain
 * a device pointer to the most recent debayered RGB frame *without* any
 * ROS serialization or host<->device copy.
 *
 * Lifecycle:
 *   1. Consumer calls get_latest_gpu_frame() -- returns a filled GpuFrame.
 *   2. Consumer uses device_ptr (read-only) for inference / ISP.
 *   3. Consumer calls release_gpu_frame(buffer_index) to return the slot.
 *
 * The device_ptr points to GPU global memory (uint8, HWC, rgb8).
 */
struct GpuFrame {
    void* device_ptr;        ///< Pointer (pinned host or GPU device memory)
    int width;               ///< Frame width in pixels
    int height;              ///< Frame height in pixels
    int channels;            ///< 1 = BayerRG8, 3 = RGB8
    std::string encoding;    ///< "bayer_rggb8" or "rgb8"
    uint64_t timestamp_ns;   ///< Capture timestamp (PTP or wall-clock)
    int buffer_index;        ///< Return to pool via release_gpu_frame()
    uint64_t frame_id;       ///< Monotonic Vimba frame counter
};

class CudaCameraNode : public rclcpp::Node {
public:
    explicit CudaCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CudaCameraNode() override;

    // ----- Zero-copy GPU frame API (for in-process composition) -----

    /**
     * Thread-safe getter for the latest debayered GPU frame.
     *
     * Returns std::nullopt when no frame is available or gpu_direct is
     * disabled.  The returned GpuFrame::device_ptr remains valid until
     * the caller invokes release_gpu_frame(buffer_index).
     */
    std::optional<GpuFrame> get_latest_gpu_frame();

    /**
     * Signal that the consumer is done reading buffer_index.
     * This allows the driver to reuse the slot for future frames.
     */
    void release_gpu_frame(int buffer_index);

private:
    // --- Camera setup ---
    void open_camera();
    void close_camera();
    void configure_camera_features();
    void start_capture();
    void stop_capture();

    // --- Vimba frame callback (runs on Vimba's internal thread) ---
    static void VMB_CALL frame_callback(VmbHandle_t camera_handle,
                                        VmbHandle_t stream_handle,
                                        VmbFrame_t* frame);

    // --- Publisher thread ---
    void publisher_loop();

    // --- Service callbacks ---
    void on_start_stream(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_stop_stream(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void on_load_settings(
        const std::shared_ptr<avt_vimba_camera_msgs::srv::LoadSettings::Request> request,
        std::shared_ptr<avt_vimba_camera_msgs::srv::LoadSettings::Response> response);
    void on_save_settings(
        const std::shared_ptr<avt_vimba_camera_msgs::srv::SaveSettings::Request> request,
        std::shared_ptr<avt_vimba_camera_msgs::srv::SaveSettings::Response> response);

    // --- Dynamic parameter callback ---
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters);

    // --- GAP 2: Dynamic feature discovery ---
    void discover_camera_features();

    // --- GAP 3: Watchdog / reconnect ---
    void watchdog_callback();
    void attempt_reconnect();

    // --- GAP 1: PCAP replay ---
    void start_pcap_replay();
    void pcap_replay_thread_func();
    void keyboard_input_thread_func();
    void setup_terminal();
    void restore_terminal();
    void print_keyboard_controls();

    // --- Parameters ---
    std::string camera_id_;
    std::string camera_info_url_;
    double frame_rate_;
    int num_buffers_;
    uint32_t width_;
    uint32_t height_;

    // Cached parameters read by the Vimba callback thread (D6 fix)
    std::atomic<bool> use_ptp_{false};
    std::atomic<int64_t> ptp_offset_{0};

    // --- Vimba state ---
    VmbHandle_t camera_handle_;
    bool camera_open_;
    std::vector<VmbFrame_t> vmb_frames_;  ///< Announced frame structures

    // --- CUDA / zero-copy pipeline ---
    std::unique_ptr<PinnedBufferPool> buffer_pool_;
    std::unique_ptr<SPSCQueue<FrameEntry>> frame_queue_;

    // GPU output buffers (allocated once)
    uint8_t* d_rgb_output_;       ///< Device memory for RGB output
    void* cuda_stream_;           ///< CUDA stream for async operations

    // --- GPU-direct zero-copy frame sharing (composition IPC) ---
    static constexpr int kMaxGpuDirectBuffers = 3;
    uint8_t* d_gpu_direct_bufs_[kMaxGpuDirectBuffers] = {};  ///< Ring of GPU RGB buffers
    std::atomic<int> gpu_direct_write_idx_{0};                ///< Next slot to write
    std::atomic<bool> gpu_direct_slot_held_[kMaxGpuDirectBuffers] = {};  ///< Consumer hold flags
    mutable std::mutex gpu_frame_mutex_;                      ///< Guards latest_gpu_frame_
    std::optional<GpuFrame> latest_gpu_frame_;                ///< Most recent frame metadata
    bool gpu_direct_enabled_ = false;                         ///< Controlled by parameter

    // v2.0.0 configurable optimization flags
    bool publish_raw_bayer_ = true;   ///< Skip debayer, publish BayerRG8 (3x smaller)
    bool roi_enabled_ = false;        ///< On-sensor ROI crop (eliminate sky)
    bool enable_ptp_sync_ = true;     ///< IEEE 1588 PTP hardware timestamp sync

    // --- ROS publishers / services ---
    image_transport::CameraPublisher camera_pub_;  ///< image_transport publisher
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
    rclcpp::Service<avt_vimba_camera_msgs::srv::LoadSettings>::SharedPtr load_srv_;
    rclcpp::Service<avt_vimba_camera_msgs::srv::SaveSettings>::SharedPtr save_srv_;

    // --- Publisher thread ---
    std::thread pub_thread_;
    std::atomic<bool> running_;

    // --- Parameter callback handle ---
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // --- GAP 1: PCAP replay state ---
    bool enable_pcap_{false};
    std::string pcap_file_path_;
    std::shared_ptr<PcapReader> pcap_reader_;
    std::thread pcap_thread_;
    std::atomic<bool> pcap_thread_running_{false};
    std::atomic<int> pcap_frame_index_{0};

    // PCAP keyboard control
    std::atomic<double> pcap_playback_speed_{1.0};
    std::atomic<bool> pcap_paused_{false};
    std::atomic<bool> pcap_step_forward_{false};
    std::atomic<bool> pcap_step_backward_{false};
    std::thread keyboard_thread_;
    std::atomic<bool> keyboard_thread_running_{false};
    int tty_fd_{-1};
    struct termios orig_termios_;

    static constexpr double pcap_seek_time_ = 0.5;
    static constexpr double pcap_speed_increment_ = 0.1;
    static constexpr double pcap_speed_min_ = 0.1;
    static constexpr double pcap_speed_max_ = 10.0;
    static constexpr double pcap_assumed_fps_ = 30.0;

    // --- GAP 2: Dynamic feature discovery state ---
    std::map<std::string, bool> discovered_features_;  ///< feature name -> writable

    // --- GAP 3: Watchdog state ---
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    std::atomic<uint64_t> last_frame_time_ns_{0};  ///< epoch ns of last received frame
    static constexpr double watchdog_timeout_sec_ = 2.0;
};

}  // namespace avt_vimba_camera
