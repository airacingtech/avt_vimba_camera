/// Copyright (c) 2026, AI Racing Tech

#include "avt_vimba_camera/auto_exposure.hpp"

#include <algorithm>
#include <cmath>

namespace avt_vimba_camera
{
namespace
{

double Clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/// Bytes per pixel for the encodings this driver can deliver. Returns 0 for anything whose
/// first byte is not a usable luminance proxy, which makes Update() decline the frame rather
/// than compute a metric over misinterpreted bytes.
int BytesPerPixel(const std::string& encoding)
{
  if (encoding == "mono8" || encoding.compare(0, 6, "bayer_") == 0)
  {
    return 1;
  }
  if (encoding == "rgb8" || encoding == "bgr8")
  {
    return 3;
  }
  if (encoding == "rgba8" || encoding == "bgra8")
  {
    return 4;
  }
  return 0;
}

double GainDbToLinear(double db)
{
  return std::pow(10.0, db / 20.0);
}

double GainLinearToDb(double linear)
{
  return 20.0 * std::log10(std::max(linear, 1e-9));
}

}  // namespace

double ExposureBlurBudgetUs(double smear_px, double distance_m, double focal_px, double speed_mps)
{
  if (speed_mps <= 1e-6 || focal_px <= 0.0)
  {
    // A stationary car spends no blur budget, so nothing here constrains the exposure.
    return 1e9;
  }
  return 1e6 * smear_px * distance_m / (focal_px * speed_mps);
}

AutoExposure::AutoExposure(const AutoExposureConfig& config, double initial_exposure_us,
                           double initial_gain_db)
  : config_(config)
{
  // An odd stride would walk across Bayer colour phases as it steps along a row, turning the
  // colour mosaic itself into the dominant "gradient" in the sampled image.
  if (config_.sample_stride < 2)
  {
    config_.sample_stride = 2;
  }
  if (config_.sample_stride % 2 != 0)
  {
    config_.sample_stride += 1;
  }
  if (config_.gamma <= 1.0)
  {
    config_.gamma = 1.9;
  }
  if (config_.max_step_ratio < 1.0)
  {
    config_.max_step_ratio = 1.0;
  }

  exposure_us_ = Clamp(initial_exposure_us, config_.exposure_min_us, config_.exposure_max_us);
  gain_db_ = Clamp(initial_gain_db, config_.gain_min_db, config_.gain_max_db);

  if (config_.probe_model != "gamma" && config_.probe_model != "linear")
  {
    config_.probe_model = "linear";
  }
  if (config_.probe_step_db <= 0.0)
  {
    config_.probe_step_db = 4.0;
  }

  BuildGammaLut(1.0, lut_identity_);
  BuildProbeLuts();
}

void AutoExposure::BuildProbeLuts()
{
  if (config_.probe_model == "gamma")
  {
    // gamma > 1 pulls values down (i/255 < 1), which is what a shorter exposure looks like.
    BuildGammaLut(config_.gamma, lut_darker_);
    BuildGammaLut(1.0 / config_.gamma, lut_brighter_);
    return;
  }
  const double scale = std::pow(10.0, config_.probe_step_db / 20.0);
  BuildLinearLut(scale, lut_brighter_);
  BuildLinearLut(1.0 / scale, lut_darker_);
}

void AutoExposure::BuildLinearLut(double scale, uint8_t* lut)
{
  for (int i = 0; i < 256; ++i)
  {
    lut[i] = static_cast<uint8_t>(Clamp(std::round(i * scale), 0.0, 255.0));
  }
}

void AutoExposure::BuildGammaLut(double gamma, uint8_t* lut)
{
  for (int i = 0; i < 256; ++i)
  {
    const double v = 255.0 * std::pow(static_cast<double>(i) / 255.0, gamma);
    lut[i] = static_cast<uint8_t>(Clamp(std::lround(v), 0.0, 255.0));
  }
}

void AutoExposure::SyncFromCamera(double exposure_us, double gain_db)
{
  exposure_us_ = Clamp(exposure_us, config_.exposure_min_us, config_.exposure_max_us);
  gain_db_ = Clamp(gain_db, config_.gain_min_db, config_.gain_max_db);
}

bool AutoExposure::Sample(const uint8_t* data, uint32_t width, uint32_t height, uint32_t step,
                          const std::string& encoding)
{
  const int bpp = BytesPerPixel(encoding);
  if (data == nullptr || bpp == 0 || width == 0 || height == 0)
  {
    return false;
  }

  const int stride = config_.sample_stride;
  gray_w_ = static_cast<int>(width) / stride;
  gray_h_ = static_cast<int>(height) / stride;
  // The metric uses central differences, so it needs at least a one-pixel interior.
  if (gray_w_ < 3 || gray_h_ < 3)
  {
    return false;
  }

  gray_.resize(static_cast<size_t>(gray_w_) * static_cast<size_t>(gray_h_));
  clip_.resize(gray_.size());

  const bool bayer = encoding.compare(0, 6, "bayer_") == 0;
  for (int y = 0; y < gray_h_; ++y)
  {
    const size_t row0 = static_cast<size_t>(y) * stride * step;
    uint8_t* dst = gray_.data() + static_cast<size_t>(y) * gray_w_;
    uint8_t* clip = clip_.data() + static_cast<size_t>(y) * gray_w_;
    for (int x = 0; x < gray_w_; ++x)
    {
      const size_t col0 = static_cast<size_t>(x) * stride * bpp;
      if (bayer)
      {
        // 2x2 mosaic cell. Whatever the phase order, one of these four is green-ish and the
        // brightest of the four is what saturation must be judged on.
        const uint8_t a = data[row0 + col0];
        const uint8_t b = data[row0 + col0 + 1];
        const uint8_t c = data[row0 + step + col0];
        const uint8_t d = data[row0 + step + col0 + 1];
        // RGGB / BGGR / GRBG / GBRG all place a green at one of the two off-diagonal slots;
        // averaging them is a stable luminance proxy without needing to know the phase.
        dst[x] = static_cast<uint8_t>((static_cast<int>(b) + c) / 2);
        clip[x] = std::max(std::max(a, b), std::max(c, d));
      }
      else
      {
        dst[x] = data[row0 + col0];
        clip[x] = dst[x];
      }
    }
  }
  return true;
}

double AutoExposure::Metric(const uint8_t* lut) const
{
  const int w = gray_w_;
  const int h = gray_h_;
  if (w < 3 || h < 3)
  {
    return 0.0;
  }

  const double lambda = config_.grad_lambda;
  const double threshold = config_.grad_threshold;
  // Normalizing by the interior pixel COUNT rather than by the number of pixels that cleared
  // the threshold is deliberate. Dividing by the survivor count would let a frame that blows
  // out most of the scene but keeps a few hard edges score as well as one that keeps structure
  // everywhere, which is precisely the failure we are trying to move away from.
  const double n = static_cast<double>((w - 2) * (h - 2));
  const double norm = std::log(lambda * (1.0 - threshold) + 1.0);
  if (n <= 0.0 || norm <= 0.0)
  {
    return 0.0;
  }

  double acc = 0.0;
  for (int y = 1; y < h - 1; ++y)
  {
    const uint8_t* row = gray_.data() + static_cast<size_t>(y) * w;
    const uint8_t* up = row - w;
    const uint8_t* down = row + w;
    for (int x = 1; x < w - 1; ++x)
    {
      const double gx = (static_cast<double>(lut[row[x + 1]]) - lut[row[x - 1]]) / (2.0 * 255.0);
      const double gy = (static_cast<double>(lut[down[x]]) - lut[up[x]]) / (2.0 * 255.0);
      const double m = std::sqrt(gx * gx + gy * gy);
      if (m > threshold)
      {
        acc += std::log(lambda * (m - threshold) + 1.0);
      }
    }
  }
  return acc / (n * norm);
}

double AutoExposure::MetricForTest(const std::vector<uint8_t>& gray, int w, int h,
                                   double gamma) const
{
  AutoExposure copy(*this);
  copy.gray_ = gray;
  copy.gray_w_ = w;
  copy.gray_h_ = h;
  uint8_t lut[256];
  BuildGammaLut(gamma, lut);
  return copy.Metric(lut);
}

void AutoExposure::ApplyLightFactor(double factor)
{
  const double gain_lin_min = GainDbToLinear(config_.gain_min_db);
  const double gain_lin_max = GainDbToLinear(config_.gain_max_db);

  const double current_light = exposure_us_ * GainDbToLinear(gain_db_);
  const double target_light = current_light * factor;

  // Exposure absorbs the demand first, up to the blur/frame-rate budget, because integrating
  // longer is noise-free while gain amplifies read noise along with signal. Gain only picks up
  // what exposure is not allowed to. Run in reverse this also means gain is the first thing
  // given back when the scene brightens, which is what we want: the noisiest term goes first.
  const double exposure = Clamp(target_light, config_.exposure_min_us, config_.exposure_max_us);
  const double gain_lin = Clamp(target_light / exposure, gain_lin_min, gain_lin_max);

  exposure_us_ = exposure;
  gain_db_ = Clamp(GainLinearToDb(gain_lin), config_.gain_min_db, config_.gain_max_db);
}

bool AutoExposure::Update(const uint8_t* data, uint32_t width, uint32_t height, uint32_t step,
                          const std::string& encoding, double now_s, AutoExposureCommand* out)
{
  if (!config_.enabled || out == nullptr)
  {
    return false;
  }

  // Rate limit before doing any work: the metric is cheap but not free, and there is no point
  // computing a correction on a cycle that is not allowed to write one.
  if (have_last_update_ && config_.update_hz > 0.0)
  {
    if (now_s - last_update_s_ < 1.0 / config_.update_hz)
    {
      return false;
    }
  }

  if (!Sample(data, width, height, step, encoding))
  {
    return false;
  }
  last_update_s_ = now_s;
  have_last_update_ = true;

  size_t clipped_low = 0;
  size_t clipped_high = 0;
  double level_sum = 0.0;
  const uint8_t low_rail = static_cast<uint8_t>(Clamp(config_.clip_low * 255.0, 0.0, 255.0));
  const uint8_t high_rail = static_cast<uint8_t>(Clamp(config_.clip_high * 255.0, 0.0, 255.0));
  for (size_t i = 0; i < clip_.size(); ++i)
  {
    if (clip_[i] <= low_rail)
    {
      ++clipped_low;
    }
    else if (clip_[i] >= high_rail)
    {
      ++clipped_high;
    }
    level_sum += gray_[i];
  }
  const double n = static_cast<double>(gray_.size());
  out->clipped_low_frac = clipped_low / n;
  out->clipped_high_frac = clipped_high / n;
  out->mean_level = level_sum / (n * 255.0);

  out->metric = Metric(lut_identity_);
  out->metric_brighter = Metric(lut_brighter_);
  out->metric_darker = Metric(lut_darker_);
  out->saturation_override = false;
  out->mean_fallback = false;

  // Clipped pixels have no gradient, so a frame that blows out the sky can score BETTER than a
  // correctly exposed one: the washed-out region stops contributing texture-free samples that
  // drag the average down. The metric cannot see its way out of that, so saturation is handled
  // as a separate hard constraint ahead of it. Only one rail can win; a genuinely high-contrast
  // scene clips at both ends and there is nothing exposure can do about it, so the metric
  // decides instead.
  const bool blown_out = out->clipped_high_frac > config_.saturation_frac_max;
  const bool crushed = out->clipped_low_frac > config_.saturation_frac_max;

  // Saturation acts as a proportional barrier, not a switch. A fixed full-size cut whenever the
  // clip budget is exceeded makes the loop limit cycle: the guard slams the gain down, the
  // metric walks it straight back up, and the image visibly pulses. Measured as 10.4 -> 12.7 ->
  // 11.6 -> 10.4 dB, which raised the mean score but LOWERED the 10th percentile, i.e. it made
  // the worst frames worse. Scaling the pushback by how far over budget we are lets the
  // controller come to rest against the boundary instead of bouncing off it.
  double saturation_drive = 0.0;
  bool saturation_active = false;
  const double limit = config_.saturation_frac_max > 1e-9 ? config_.saturation_frac_max : 0.02;
  if (blown_out && !crushed)
  {
    saturation_drive = -Clamp((out->clipped_high_frac - limit) / limit, 0.0, 1.0);
    saturation_active = true;
  }
  else if (crushed && !blown_out)
  {
    saturation_drive = Clamp((out->clipped_low_frac - limit) / limit, 0.0, 1.0);
    saturation_active = true;
  }

  double d;
  {
    const double denom = out->metric_brighter + out->metric_darker;
    if (denom > 1e-12)
    {
      d = (out->metric_brighter - out->metric_darker) / denom;
    }
    else
    {
      // No gradient survives the threshold in either probe, so the metric has no opinion and
      // its ratio is 0/0. Steer on mean level instead rather than freezing the exposure.
      out->mean_fallback = true;
      const double target =
          config_.fallback_target_mean > 1e-6 ? config_.fallback_target_mean : 0.35;
      d = Clamp((target - out->mean_level) / target, -1.0, 1.0);
    }
  }

  // The barrier can only ever push toward the interior of the budget; it never overrides the
  // metric in the direction that would make clipping worse. A scene clipping at BOTH rails is
  // beyond anything exposure can fix, so neither barrier engages and the metric decides.
  if (saturation_active)
  {
    const double constrained =
        blown_out ? std::min(d, saturation_drive) : std::max(d, saturation_drive);
    out->saturation_override = constrained != d;
    d = constrained;
  }
  out->drive = d;

  if (std::fabs(d) <= config_.deadband)
  {
    // Inside the deadband. Report the telemetry but do not spend a GigE write, and do not let
    // a string of sub-deadband nudges accumulate into a drift.
    out->exposure_us = exposure_us_;
    out->gain_db = gain_db_;
    return false;
  }
  double factor = std::exp(config_.kp * d);

  factor = Clamp(factor, 1.0 / config_.max_step_ratio, config_.max_step_ratio);

  const double prev_exposure = exposure_us_;
  const double prev_gain = gain_db_;
  ApplyLightFactor(factor);

  out->exposure_us = exposure_us_;
  out->gain_db = gain_db_;

  // Both actuators already at a rail in the requested direction: nothing changed, so there is
  // nothing to write. Without this the controller would re-send an identical setpoint every
  // cycle for as long as the scene stayed out of range.
  const bool moved = std::fabs(exposure_us_ - prev_exposure) > 1e-6 ||
                     std::fabs(gain_db_ - prev_gain) > 1e-6;
  return moved;
}

}  // namespace avt_vimba_camera
