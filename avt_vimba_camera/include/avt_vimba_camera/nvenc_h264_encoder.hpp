/// Copyright (c) 2026, AI Racing Tech
///
/// Direct NVENC H.264 encoder for the telemetry uplink, replacing the per-camera
/// isaac_ros_h264_encoder NITROS graphs. Measured on the car: each encoder graph cost
/// ~0.11 cores of framework overhead (GXF scheduler + VPI convert + NITROS pub/sub) to
/// encode a 256x192 frame whose actual NVENC work is well under a millisecond. Encoding
/// in-process on the already-resized device buffer removes all of it.

#ifndef AVT_VIMBA_CAMERA__NVENC_H264_ENCODER_HPP_
#define AVT_VIMBA_CAMERA__NVENC_H264_ENCODER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <cuda.h>
#include <nvEncodeAPI.h>

namespace avt_vimba_camera
{

/// One NVENC session encoding a fixed-size I420 stream. Not thread safe: owned and
/// driven by its camera's frame thread. Construction opens the session and allocates
/// the pitched device input buffer; Encode() is synchronous (submits and locks the
/// bitstream in place), which is fine for the uplink path where the whole per-frame
/// budget is milliseconds.
class NvencH264Encoder
{
public:
  struct Config
  {
    uint32_t width{ 0 };
    uint32_t height{ 0 };
    int32_t bitrate{ 100000 };
    int32_t max_bitrate{ 0 };      // 0: same as bitrate
    int32_t framerate{ 20 };
    int32_t iframe_interval{ 20 }; // GOP length / IDR period, frames
    int32_t intra_refresh{ 0 };    // 0: off; else rolling refresh period, frames
    int32_t vbv_buffer_frames{ 1 };
    int32_t qp{ 30 };              // used only for rate_control "cqp"
    std::string rate_control{ "cbr" };  // cbr | vbr | cqp
    // Quality knobs. All three are resolved inside NVENC and cost the host nothing, which is
    // why they are the right lever when the CPU budget is one core.
    std::string preset{ "p3" };    // p1..p7: slower preset = better compression, more encoder GPU
    int32_t aq{ 0 };               // spatial adaptive quantisation strength 1..15; 0 = off
    std::string profile{ "auto" }; // auto | baseline | main | high ("high" => CABAC + 8x8)
    // low_latency | ultra_low_latency | high_quality. Picks NVENC's tuning preset, which decides
    // how hard the rate controller is allowed to defer bits. ultra_low_latency holds every frame
    // to its own budget (lowest delay, some quality cost on scene changes); high_quality lets it
    // defer and is wrong for a live uplink.
    std::string tuning{ "low_latency" };
  };

  /// Throws std::runtime_error when NVENC cannot be opened/configured.
  NvencH264Encoder(const Config& config, CUcontext cuda_context);
  ~NvencH264Encoder();

  NvencH264Encoder(const NvencH264Encoder&) = delete;
  NvencH264Encoder& operator=(const NvencH264Encoder&) = delete;

  /// The encoder's device input buffer: write an I420 frame here (e.g. with NPP on any
  /// stream), synchronize, then call Encode(). Planes: Y at Y(), U at U(), V at V();
  /// luma rows are Pitch() bytes apart, chroma rows Pitch()/2.
  CUdeviceptr Y() const { return input_; }
  CUdeviceptr U() const { return input_ + static_cast<size_t>(pitch_) * height_; }
  CUdeviceptr V() const
  {
    return U() + static_cast<size_t>(pitch_ / 2) * (height_ / 2);
  }
  uint32_t Pitch() const { return pitch_; }

  /// Fill both chroma planes with 0x80 (grey): grayscale output without any per-frame
  /// chroma conversion. Call once after construction for monochrome streams.
  bool SetMonochrome();

  /// Encode the frame currently in the input buffer. The buffer must be idle (device
  /// work writing it completed). Appends an Annex-B access unit to `out` (cleared
  /// first). SPS/PPS are repeated on every IDR so a viewer can join mid-stream.
  bool Encode(std::vector<uint8_t>& out);

private:
  uint32_t width_{ 0 };
  uint32_t height_{ 0 };
  uint32_t pitch_{ 0 };

  CUcontext ctx_{ nullptr };
  void* session_{ nullptr };
  NV_ENCODE_API_FUNCTION_LIST fn_{};

  CUdeviceptr input_{ 0 };
  NV_ENC_REGISTERED_PTR registered_{ nullptr };
  NV_ENC_OUTPUT_PTR bitstream_{ nullptr };
};

}  // namespace avt_vimba_camera

#endif  // AVT_VIMBA_CAMERA__NVENC_H264_ENCODER_HPP_
