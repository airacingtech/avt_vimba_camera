/// Copyright (c) 2026, AI Racing Tech

#include "avt_vimba_camera/nvenc_h264_encoder.hpp"

#include <cstring>
#include <stdexcept>

namespace avt_vimba_camera
{

namespace
{

[[noreturn]] void Fail(const char* what, NVENCSTATUS status)
{
  throw std::runtime_error(std::string("NVENC ") + what + " failed with status " +
                           std::to_string(static_cast<int>(status)));
}

void Check(NVENCSTATUS status, const char* what)
{
  if (status != NV_ENC_SUCCESS)
  {
    Fail(what, status);
  }
}

/// "p1".."p7" -> preset GUID. Anything unrecognised falls back to P3, the value this encoder
/// used before the knob existed, so a typo degrades to the old behaviour instead of throwing.
NV_ENC_TUNING_INFO TuningInfo(const std::string& name)
{
  if (name == "ultra_low_latency") return NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY;
  if (name == "high_quality") return NV_ENC_TUNING_INFO_HIGH_QUALITY;
  return NV_ENC_TUNING_INFO_LOW_LATENCY;
}

GUID PresetGuid(const std::string& name)
{
  if (name == "p1") return NV_ENC_PRESET_P1_GUID;
  if (name == "p2") return NV_ENC_PRESET_P2_GUID;
  if (name == "p4") return NV_ENC_PRESET_P4_GUID;
  if (name == "p5") return NV_ENC_PRESET_P5_GUID;
  if (name == "p6") return NV_ENC_PRESET_P6_GUID;
  if (name == "p7") return NV_ENC_PRESET_P7_GUID;
  return NV_ENC_PRESET_P3_GUID;
}

}  // namespace

NvencH264Encoder::NvencH264Encoder(const Config& config, CUcontext cuda_context)
  : width_(config.width), height_(config.height), ctx_(cuda_context)
{
  if (width_ == 0 || height_ == 0 || (width_ % 2) != 0 || (height_ % 2) != 0)
  {
    throw std::runtime_error("NVENC input size must be even and non-zero");
  }

  fn_.version = NV_ENCODE_API_FUNCTION_LIST_VER;
  Check(NvEncodeAPICreateInstance(&fn_), "NvEncodeAPICreateInstance");

  NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS open{};
  open.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
  open.deviceType = NV_ENC_DEVICE_TYPE_CUDA;
  open.device = ctx_;
  open.apiVersion = NVENCAPI_VERSION;
  Check(fn_.nvEncOpenEncodeSessionEx(&open, &session_), "nvEncOpenEncodeSessionEx");

  // Preset config first, then apply the uplink's rate control on top. The same preset GUID has
  // to go to both nvEncGetEncodePresetConfigEx and nvEncInitializeEncoder -- fetching P3's config
  // and then initialising as P5 would silently mix two presets' tuning.
  const GUID preset_guid = PresetGuid(config.preset);
  // Same value must reach nvEncGetEncodePresetConfigEx and nvEncInitializeEncoder, for the same
  // reason as the preset: fetching one tuning's config and initialising with another mixes two
  // rate-control setups.
  const NV_ENC_TUNING_INFO tuning = TuningInfo(config.tuning);
  NV_ENC_PRESET_CONFIG preset{};
  preset.version = NV_ENC_PRESET_CONFIG_VER;
  preset.presetCfg.version = NV_ENC_CONFIG_VER;
  Check(fn_.nvEncGetEncodePresetConfigEx(session_, NV_ENC_CODEC_H264_GUID, preset_guid,
                                         tuning, &preset),
        "nvEncGetEncodePresetConfigEx");
  NV_ENC_CONFIG enc_cfg = preset.presetCfg;

  const uint32_t gop = config.iframe_interval > 0 ?
      static_cast<uint32_t>(config.iframe_interval) : 20u;
  enc_cfg.gopLength = gop;
  enc_cfg.frameIntervalP = 1;  // IPPP..., no B frames on the low-latency uplink
  enc_cfg.encodeCodecConfig.h264Config.idrPeriod = gop;
  enc_cfg.encodeCodecConfig.h264Config.repeatSPSPPS = 1;

  // High profile is a straight compression win at this bitrate: it turns on CABAC (entropy
  // coding, worth roughly 5-10% bitrate over CAVLC on its own) and the adaptive 8x8 transform,
  // which suits the large flat regions -- tarmac, sky, wall -- that dominate these views. Left
  // at "auto" the preset picks, which is not guaranteed to be High. Baseline forbids CABAC, so
  // entropy coding is only forced when the profile actually permits it.
  if (config.profile == "high" || config.profile == "main")
  {
    enc_cfg.profileGUID = config.profile == "high" ? NV_ENC_H264_PROFILE_HIGH_GUID
                                                  : NV_ENC_H264_PROFILE_MAIN_GUID;
    enc_cfg.encodeCodecConfig.h264Config.entropyCodingMode =
        NV_ENC_H264_ENTROPY_CODING_MODE_CABAC;
    if (config.profile == "high")
    {
      enc_cfg.encodeCodecConfig.h264Config.adaptiveTransformMode =
          NV_ENC_H264_ADAPTIVE_TRANSFORM_ENABLE;
    }
  }
  else if (config.profile == "baseline")
  {
    enc_cfg.profileGUID = NV_ENC_H264_PROFILE_BASELINE_GUID;
  }
  if (config.intra_refresh > 0)
  {
    enc_cfg.encodeCodecConfig.h264Config.enableIntraRefresh = 1;
    enc_cfg.encodeCodecConfig.h264Config.intraRefreshPeriod =
        static_cast<uint32_t>(config.intra_refresh);
    enc_cfg.encodeCodecConfig.h264Config.intraRefreshCnt =
        static_cast<uint32_t>(config.intra_refresh > 1 ? config.intra_refresh - 1 : 1);
  }

  NV_ENC_RC_PARAMS& rc = enc_cfg.rcParams;
  const uint32_t fps = config.framerate > 0 ? static_cast<uint32_t>(config.framerate) : 20u;
  if (config.rate_control == "cqp")
  {
    rc.rateControlMode = NV_ENC_PARAMS_RC_CONSTQP;
    rc.constQP.qpInterP = rc.constQP.qpInterB = rc.constQP.qpIntra =
        static_cast<uint32_t>(config.qp);
  }
  else
  {
    rc.rateControlMode =
        config.rate_control == "vbr" ? NV_ENC_PARAMS_RC_VBR : NV_ENC_PARAMS_RC_CBR;
    rc.averageBitRate = static_cast<uint32_t>(config.bitrate);
    rc.maxBitRate = static_cast<uint32_t>(
        config.max_bitrate > 0 ? config.max_bitrate : config.bitrate);
    const uint32_t frames = config.vbv_buffer_frames > 0 ?
        static_cast<uint32_t>(config.vbv_buffer_frames) : 1u;
    rc.vbvBufferSize = rc.averageBitRate * frames / fps;
    rc.vbvInitialDelay = rc.vbvBufferSize;

    // Spatial AQ only. Temporal AQ is deliberately not offered: it needs lookahead, and any
    // lookahead at 10 fps costs 100 ms of latency per frame of depth on a link whose whole
    // point is being current. aqStrength is a 4-bit field, so 1..15; 0 leaves AQ off entirely.
    if (config.aq > 0)
    {
      rc.enableAQ = 1;
      rc.aqStrength = static_cast<uint32_t>(config.aq > 15 ? 15 : config.aq);
    }
  }

  NV_ENC_INITIALIZE_PARAMS init{};
  init.version = NV_ENC_INITIALIZE_PARAMS_VER;
  init.encodeGUID = NV_ENC_CODEC_H264_GUID;
  init.presetGUID = preset_guid;
  init.tuningInfo = tuning;
  init.encodeWidth = width_;
  init.encodeHeight = height_;
  init.darWidth = width_;
  init.darHeight = height_;
  init.frameRateNum = fps;
  init.frameRateDen = 1;
  init.enablePTD = 1;
  init.encodeConfig = &enc_cfg;
  Check(fn_.nvEncInitializeEncoder(session_, &init), "nvEncInitializeEncoder");

  // Single pitched I420 input allocation: Y (pitch x H), U and V (pitch/2 x H/2) packed
  // behind it. cuMemAllocPitch's alignment covers the pitch/2 chroma rows too.
  size_t pitch = 0;
  CUresult cu = cuMemAllocPitch(&input_, &pitch, width_,
                                static_cast<size_t>(height_) + height_ / 2 + 1, 16);
  if (cu != CUDA_SUCCESS)
  {
    throw std::runtime_error("cuMemAllocPitch for NVENC input failed");
  }
  pitch_ = static_cast<uint32_t>(pitch);

  NV_ENC_REGISTER_RESOURCE reg{};
  reg.version = NV_ENC_REGISTER_RESOURCE_VER;
  reg.resourceType = NV_ENC_INPUT_RESOURCE_TYPE_CUDADEVICEPTR;
  reg.resourceToRegister = reinterpret_cast<void*>(input_);
  reg.width = width_;
  reg.height = height_;
  reg.pitch = pitch_;
  reg.bufferFormat = NV_ENC_BUFFER_FORMAT_IYUV;
  reg.bufferUsage = NV_ENC_INPUT_IMAGE;
  Check(fn_.nvEncRegisterResource(session_, &reg), "nvEncRegisterResource");
  registered_ = reg.registeredResource;

  NV_ENC_CREATE_BITSTREAM_BUFFER bs{};
  bs.version = NV_ENC_CREATE_BITSTREAM_BUFFER_VER;
  Check(fn_.nvEncCreateBitstreamBuffer(session_, &bs), "nvEncCreateBitstreamBuffer");
  bitstream_ = bs.bitstreamBuffer;
}

NvencH264Encoder::~NvencH264Encoder()
{
  if (session_ != nullptr)
  {
    if (bitstream_ != nullptr)
    {
      fn_.nvEncDestroyBitstreamBuffer(session_, bitstream_);
    }
    if (registered_ != nullptr)
    {
      fn_.nvEncUnregisterResource(session_, registered_);
    }
    fn_.nvEncDestroyEncoder(session_);
  }
  if (input_ != 0)
  {
    cuMemFree(input_);
  }
}

bool NvencH264Encoder::SetMonochrome()
{
  const size_t chroma_bytes =
      static_cast<size_t>(pitch_ / 2) * (height_ / 2) * 2;
  return cuMemsetD8(U(), 0x80, chroma_bytes) == CUDA_SUCCESS;
}

bool NvencH264Encoder::Encode(std::vector<uint8_t>& out)
{
  out.clear();

  NV_ENC_MAP_INPUT_RESOURCE map{};
  map.version = NV_ENC_MAP_INPUT_RESOURCE_VER;
  map.registeredResource = registered_;
  if (fn_.nvEncMapInputResource(session_, &map) != NV_ENC_SUCCESS)
  {
    return false;
  }

  NV_ENC_PIC_PARAMS pic{};
  pic.version = NV_ENC_PIC_PARAMS_VER;
  pic.inputBuffer = map.mappedResource;
  pic.bufferFmt = map.mappedBufferFmt;
  pic.inputWidth = width_;
  pic.inputHeight = height_;
  pic.inputPitch = pitch_;
  pic.outputBitstream = bitstream_;
  pic.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
  const NVENCSTATUS enc_status = fn_.nvEncEncodePicture(session_, &pic);

  bool ok = enc_status == NV_ENC_SUCCESS;
  if (ok)
  {
    NV_ENC_LOCK_BITSTREAM lock{};
    lock.version = NV_ENC_LOCK_BITSTREAM_VER;
    lock.outputBitstream = bitstream_;
    ok = fn_.nvEncLockBitstream(session_, &lock) == NV_ENC_SUCCESS;
    if (ok)
    {
      const auto* data = static_cast<const uint8_t*>(lock.bitstreamBufferPtr);
      out.assign(data, data + lock.bitstreamSizeInBytes);
      fn_.nvEncUnlockBitstream(session_, bitstream_);
    }
  }

  fn_.nvEncUnmapInputResource(session_, map.mappedResource);
  return ok;
}

}  // namespace avt_vimba_camera
