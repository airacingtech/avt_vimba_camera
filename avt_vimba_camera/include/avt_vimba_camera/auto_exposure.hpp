/// Copyright (c) 2026, AI Racing Tech

#ifndef AVT_VIMBA_CAMERA__AUTO_EXPOSURE_HPP_
#define AVT_VIMBA_CAMERA__AUTO_EXPOSURE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace avt_vimba_camera
{

/// Gradient-metric auto exposure.
///
/// The camera's own AE (`ExposureAuto: Continuous`, `ExposureAutoAlg: FitRange`) drives the
/// image toward a mean-intensity target. Mean intensity is the wrong objective for a detector:
/// it is happy to wash out a bright sky or crush a shadowed grandstand as long as the average
/// lands on target, and both destroy the edge structure bbox regression and VO depend on.
/// This controller maximizes a saturation-penalized image-gradient score instead, so exposure
/// is chosen to preserve the most usable structure rather than the prettiest histogram.
///
/// The estimate never needs to physically sweep exposures. A gamma map on the current frame
/// synthesizes what the scene would look like at a longer or shorter exposure, so one delivered
/// frame yields both a "brighter" and a "darker" probe; the sign of the difference in their
/// metrics is the direction to move. This is the gamma-probe idea from the gradient-based
/// outdoor exposure control literature (Shim et al.).
///
/// Two properties of this specific camera shape the actuator policy:
///   * The IMX265 is global shutter, so exposure costs motion blur but never rolling-shutter
///     skew. Blur is irreversible and destroys exactly the corner structure we are optimizing
///     for, whereas short-exposure sensor noise is zero-mean and survives averaging.
///   * Exposure at or above the frame period silently throttles frame rate.
/// So `exposure_max_us` is a hard blur/rate budget, and light demand is served by exposure only
/// up to that budget, with gain taking over beyond it. See ExposureBlurBudgetUs() for sizing it.
struct AutoExposureConfig
{
  /// Master switch. Off means this class is never consulted and the camera keeps whatever AE
  /// its settings XML configured, so enabling in-driver AE is always an explicit choice.
  bool enabled{ false };

  /// Gradient magnitudes below this (in normalized 0..1 units per pixel step) are treated as
  /// texture-free and excluded, which keeps flat sky and flat tarmac from dominating the score.
  double grad_threshold{ 0.06 };

  /// The log compression constant in log(lambda * (m - threshold) + 1). Larger values weight
  /// weak-but-real edges more heavily relative to strong ones.
  double grad_lambda{ 1000.0 };

  /// How a change of exposure is synthesized from the delivered frame.
  ///
  /// "linear" scales pixel values by a factor and clips, which is what a gain or exposure
  /// change physically does on a linear-response sensor. "gamma" maps through I^g, which is
  /// the model the gradient-exposure literature uses and is right for a camera with a
  /// nonlinear response curve.
  ///
  /// Default is linear because it was measured on this camera. Predicting the metric at 12 dB
  /// from an 8 dB frame, against a live sweep: linear predicted 0.1207 versus 0.1214 measured
  /// (0.6% error), while gamma predicted 0.0557 -- off by 2.2x, and ordered 4 dB above 12 dB
  /// when the truth is the reverse. The gamma probe's drive signal collapsed near 8 dB and
  /// went negative at 12 dB while the real metric was still climbing, which stalled the
  /// controller several dB short of the optimum. Keep "gamma" available for a camera whose
  /// response is genuinely nonlinear.
  std::string probe_model{ "linear" };

  /// Size of the synthetic exposure step, in dB of light, for the linear probe. Large enough
  /// to see past frame-to-frame noise, small enough that clipping in the probe still resembles
  /// clipping at the real setpoint.
  double probe_step_db{ 4.0 };

  /// Probe factor for the "gamma" model. The metric is evaluated on the frame mapped through
  /// gamma and 1/gamma, standing in for a shorter and a longer exposure respectively. Must be
  /// > 1. Unused when probe_model is "linear".
  double gamma{ 1.9 };

  /// Actuator range. exposure_max_us is the motion-blur and frame-rate budget, NOT a sensor
  /// limit; see ExposureBlurBudgetUs(). gain_max_db past ~24 dB buys noise, not signal.
  double exposure_min_us{ 20.0 };
  double exposure_max_us{ 2000.0 };
  double gain_min_db{ 0.0 };
  double gain_max_db{ 24.0 };

  /// Proportional gain on the log-exposure step. The update is multiplicative
  /// (exposure *= exp(kp * drive)), so this is unitless and scale-free across the range. With
  /// drive bounded in [-1, 1], kp is roughly "log of the largest single step wanted".
  double kp{ 0.30 };

  /// Deadband on the NORMALIZED drive signal, not on a raw metric difference. Without a
  /// deadband the controller dithers the exposure register every cycle, and every write is a
  /// GigE round trip; with one expressed in absolute metric units it would be scene dependent
  /// and silently stall on low-texture scenes.
  ///
  /// Sized against a measured exposure sweep rather than guessed. The metric's slope near the
  /// optimum is shallow, so the drive signal is already small while there is still real score
  /// left to gain: at 0.05 the controller stopped at a drive of 0.033, several dB short of the
  /// saturation limit and well short of the sweep's best in-budget point. Too small instead and
  /// the controller dithers, so this trades a little chatter for the last few dB.
  double deadband{ 0.015 };

  /// Per-update multiplicative clamp on total light. Bounds how fast the image can change when
  /// the car crosses a shadow line, so a single bad frame can never slam the exposure.
  double max_step_ratio{ 1.30 };

  /// Actuator write rate, Hz. Feature writes are GigE round trips on the same link carrying
  /// image data, and six cameras at frame rate would be a meaningful number of them, so the
  /// controller runs well below frame rate. The metric is only computed on cycles that can act.
  double update_hz{ 10.0 };

  /// Row/column decimation of the delivered frame before the metric is computed. Must be even
  /// so that a Bayer frame is sampled at a single, consistent colour phase. 8 turns a
  /// 2064x1544 frame into 258x193, which is ~50k samples: enough for a global gradient
  /// statistic, cheap enough to run on the frame callback thread.
  int sample_stride{ 8 };

  /// If more than this fraction of samples are clipped at either rail, the gradient metric is
  /// no longer trustworthy (clipped regions have zero gradient, so blowing out the sky can
  /// *raise* the score) and the controller forces a correction instead of following the metric.
  ///
  /// Measured, not guessed. Sweeping gain on the real camera: entropy plateaus at 10-11 dB
  /// (7.47 bits) but costs 13-20% of cells; 9 dB gives 7.44 bits for 5.0%, and 8 dB gives
  /// 7.33 for 1.0%. Going 9 -> 10 dB buys 0.4% more entropy for eight more points of corrupted
  /// colour, so the knee is at about 5%.
  double saturation_frac_max{ 0.05 };
  double clip_low{ 4.0 / 255.0 };
  double clip_high{ 251.0 / 255.0 };

  /// Mean level, 0..1, targeted by the fallback controller. The gradient metric ignores
  /// gradients below grad_threshold, so a scene with no texture anywhere -- fog, an unlit
  /// track, a lens cap -- scores zero on BOTH probes and yields a drive of exactly zero, which
  /// would leave the exposure frozen wherever it happened to be. When there is no gradient
  /// signal at all there is nothing to maximize, so the controller degenerates to classic
  /// mean-intensity AE until some structure reappears and the metric takes over again.
  double fallback_target_mean{ 0.35 };
};

/// What the controller wants written to the camera, plus the telemetry behind the decision.
struct AutoExposureCommand
{
  double exposure_us{ 0.0 };
  double gain_db{ 0.0 };
  /// Metric of the frame as delivered, in 0..1. Higher is more usable structure.
  double metric{ 0.0 };
  /// Metric of the synthesized brighter and darker probes.
  double metric_brighter{ 0.0 };
  double metric_darker{ 0.0 };
  /// The normalized drive signal actually acted on:
  /// (metric_brighter - metric_darker) / (metric_brighter + metric_darker), in [-1, 1].
  /// Positive means a longer exposure is predicted to preserve more structure.
  double drive{ 0.0 };
  /// Fraction of samples clipped low / high.
  double clipped_low_frac{ 0.0 };
  double clipped_high_frac{ 0.0 };
  /// True when the saturation guard overrode the metric this cycle.
  bool saturation_override{ false };
  /// True when neither probe found any gradient and the mean-intensity fallback drove instead.
  bool mean_fallback{ false };
  /// Mean sample level, 0..1.
  double mean_level{ 0.0 };
};

/// Exposure, in microseconds, at which a feature `distance_m` away smears `smear_px` pixels
/// while the car travels at `speed_mps`: t = smear * distance / (focal_px * speed).
///
/// This is the number `exposure_max_us` should be set from, and it is unforgiving at racing
/// speed: with focal_px ~1026 (about 90 deg HFOV on this sensor) at 89 m/s (200 mph), a feature
/// 25 m out smears 3.7 px in 1 ms and 110 px in 30 ms. Returns a very large value when
/// speed_mps is ~0, since a stationary car has no motion-blur budget to spend.
double ExposureBlurBudgetUs(double smear_px, double distance_m, double focal_px, double speed_mps);

/// Stateful controller. One instance per camera; not thread safe, but only ever touched from
/// the frame callback of its own camera.
class AutoExposure
{
public:
  AutoExposure(const AutoExposureConfig& config, double initial_exposure_us,
               double initial_gain_db);

  /// Feed one delivered frame. Returns true when `out` holds a new setpoint that should be
  /// written to the camera; false means the rate limiter skipped this frame, the frame was
  /// unusable, or the metric was inside the deadband and nothing needs to change.
  ///
  /// `encoding` is a ROS image encoding string; only the bytes-per-pixel and the "first byte of
  /// each pixel is luminance-ish" property are used, which holds for mono8, bayer_*8 (at an even
  /// stride) and the 8-bit interleaved colour encodings.
  bool Update(const uint8_t* data, uint32_t width, uint32_t height, uint32_t step,
              const std::string& encoding, double now_s, AutoExposureCommand* out);

  /// Current setpoint, whether or not it was accepted by the camera.
  double exposure_us() const { return exposure_us_; }
  double gain_db() const { return gain_db_; }

  /// Adopt what the camera actually reports, so a clamp or a rejected write does not leave the
  /// controller integrating against a setpoint the hardware never took.
  void SyncFromCamera(double exposure_us, double gain_db);

  const AutoExposureConfig& config() const { return config_; }

  /// Exposed for tests: the saturation-penalized gradient score of an 8-bit single-channel
  /// buffer, mapped through `gamma` first. Returns 0 for a buffer with no gradient above the
  /// threshold.
  double MetricForTest(const std::vector<uint8_t>& gray, int w, int h, double gamma) const;

private:
  /// Decimate the frame into gray_ (the luminance proxy the metric runs on) and clip_ (the
  /// per-site worst channel, used for saturation). Returns false if the geometry or encoding
  /// is unusable.
  ///
  /// These are two different samples on purpose. On a Bayer frame an even stride lands on one
  /// colour phase forever, so a metric sampled at (0,0) of an RGGB mosaic sees only red -- and
  /// is therefore blind to the blue channel clipping first, which is exactly what happens here
  /// (measured B/G = 1.23). Structure is read from green, which is the best luminance proxy and
  /// the densest phase; saturation is read as the max over the 2x2 cell, so whichever channel
  /// hits the rail first is the one the budget is enforced against.
  bool Sample(const uint8_t* data, uint32_t width, uint32_t height, uint32_t step,
              const std::string& encoding);

  /// Gradient metric over gray_ with a precomputed 256-entry gamma LUT.
  double Metric(const uint8_t* lut) const;

  /// Fill a 256-entry LUT with 255 * (i/255)^gamma.
  static void BuildGammaLut(double gamma, uint8_t* lut);

  /// Fill a 256-entry LUT with min(255, i * scale) -- what multiplying the light by `scale`
  /// does to a linear-response sensor, clipping included.
  static void BuildLinearLut(double scale, uint8_t* lut);

  /// Build the brighter/darker probe LUTs according to config_.probe_model.
  void BuildProbeLuts();

  /// Split a multiplicative light demand across exposure then gain, honouring the blur budget.
  void ApplyLightFactor(double factor);

  AutoExposureConfig config_;
  double exposure_us_;
  double gain_db_;

  std::vector<uint8_t> gray_;
  std::vector<uint8_t> clip_;
  int gray_w_{ 0 };
  int gray_h_{ 0 };

  uint8_t lut_identity_[256];
  uint8_t lut_brighter_[256];
  uint8_t lut_darker_[256];

  double last_update_s_{ 0.0 };
  bool have_last_update_{ false };
};

}  // namespace avt_vimba_camera

#endif  // AVT_VIMBA_CAMERA__AUTO_EXPOSURE_HPP_
