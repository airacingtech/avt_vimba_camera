/// Copyright (c) 2026, AI Racing Tech

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "avt_vimba_camera/auto_exposure.hpp"

namespace avt_vimba_camera
{
namespace
{

constexpr int kW = 256;
constexpr int kH = 192;

/// A smoothly textured synthetic frame at a chosen mean and contrast. A diagonal sinusoid is
/// used rather than a checkerboard because a checkerboard whose period divides the central
/// difference stencil produces zero gradient everywhere, which would test nothing.
std::vector<uint8_t> Textured(double mean, double amplitude, int w = kW, int h = kH)
{
  std::vector<uint8_t> img(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const double v = mean + amplitude * std::sin(2.0 * M_PI * (0.13 * x + 0.07 * y));
      img[static_cast<size_t>(y) * w + x] =
          static_cast<uint8_t>(std::min(255.0, std::max(0.0, std::round(v))));
    }
  }
  return img;
}

/// A dim, low-contrast scene whose gradients sit mostly below the metric's threshold, so the
/// absolute metric lands in the 1e-3 range the way a garage wall does on the real camera.
std::vector<uint8_t> LowTexture(double mean, double amplitude, int w = kW, int h = kH)
{
  std::vector<uint8_t> img(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      const double v = mean + amplitude * std::sin(2.0 * M_PI * (0.03 * x + 0.015 * y));
      img[static_cast<size_t>(y) * w + x] =
          static_cast<uint8_t>(std::min(255.0, std::max(0.0, std::round(v))));
    }
  }
  return img;
}

std::vector<uint8_t> Flat(uint8_t value, int w = kW, int h = kH)
{
  return std::vector<uint8_t>(static_cast<size_t>(w) * h, value);
}

AutoExposureConfig EnabledConfig()
{
  AutoExposureConfig c;
  c.enabled = true;
  // Stride 2 so the small synthetic frames keep a usable interior.
  c.sample_stride = 2;
  // Tests drive the controller cycle by cycle; the rate limiter is exercised separately.
  c.update_hz = 0.0;
  return c;
}

}  // namespace

// --- Blur budget -----------------------------------------------------------------------------

/// Reproduces the sizing figures for this camera: fx ~1026 px, 200 mph, a feature 25 m out.
TEST(BlurBudget, MatchesMakoG319Figures)
{
  const double focal = 1026.0;
  const double speed = 89.4;  // 200 mph
  const double distance = 25.0;

  // 3.7 px of smear should cost about 1 ms of exposure.
  EXPECT_NEAR(ExposureBlurBudgetUs(3.7, distance, focal, speed), 1000.0, 60.0);
  // and twice the smear should cost twice the exposure.
  EXPECT_NEAR(ExposureBlurBudgetUs(7.3, distance, focal, speed), 2000.0, 100.0);
}

TEST(BlurBudget, StationaryCarIsUnconstrained)
{
  EXPECT_GT(ExposureBlurBudgetUs(2.0, 25.0, 1026.0, 0.0), 1e8);
}

TEST(BlurBudget, ScalesWithDistanceAndSpeed)
{
  const double base = ExposureBlurBudgetUs(2.0, 25.0, 1026.0, 40.0);
  EXPECT_NEAR(ExposureBlurBudgetUs(2.0, 50.0, 1026.0, 40.0), 2.0 * base, 1e-6);
  EXPECT_NEAR(ExposureBlurBudgetUs(2.0, 25.0, 1026.0, 80.0), 0.5 * base, 1e-6);
}

// --- Metric ----------------------------------------------------------------------------------

TEST(Metric, FlatImageScoresZero)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  EXPECT_DOUBLE_EQ(ae.MetricForTest(Flat(128), kW, kH, 1.0), 0.0);
}

TEST(Metric, TexturedImageScoresAboveFlat)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  EXPECT_GT(ae.MetricForTest(Textured(128, 60), kW, kH, 1.0), 0.0);
}

TEST(Metric, MoreContrastScoresHigher)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const double low = ae.MetricForTest(Textured(128, 30), kW, kH, 1.0);
  const double high = ae.MetricForTest(Textured(128, 90), kW, kH, 1.0);
  EXPECT_GT(high, low);
}

/// The point of the normalization choice: a frame that keeps structure across the whole scene
/// must beat one that clips most of the scene away, even though the clipped one has stronger
/// edges where it still has any.
TEST(Metric, WidespreadStructureBeatsMostlyClipped)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const double everywhere = ae.MetricForTest(Textured(128, 60), kW, kH, 1.0);

  std::vector<uint8_t> mostly_clipped = Textured(128, 60);
  // Blow out all but a horizontal band.
  for (int y = 0; y < kH; ++y)
  {
    if (y > kH / 8)
    {
      for (int x = 0; x < kW; ++x)
      {
        mostly_clipped[static_cast<size_t>(y) * kW + x] = 255;
      }
    }
  }
  EXPECT_GT(everywhere, ae.MetricForTest(mostly_clipped, kW, kH, 1.0));
}

// --- Gamma probe direction -------------------------------------------------------------------

TEST(GammaProbe, UnderexposedFramePrefersBrighter)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> dark = Textured(20, 15);
  const double brighter = ae.MetricForTest(dark, kW, kH, 1.0 / 1.9);
  const double darker = ae.MetricForTest(dark, kW, kH, 1.9);
  EXPECT_GT(brighter, darker);
}

TEST(GammaProbe, OverexposedFramePrefersDarker)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> bright = Textured(235, 15);
  const double brighter = ae.MetricForTest(bright, kW, kH, 1.0 / 1.9);
  const double darker = ae.MetricForTest(bright, kW, kH, 1.9);
  EXPECT_GT(darker, brighter);
}

// --- Probe model ------------------------------------------------------------------------------

TEST(ProbeModel, DefaultsToLinear)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  EXPECT_EQ(ae.config().probe_model, "linear");
}

TEST(ProbeModel, UnknownModelFallsBackToLinear)
{
  AutoExposureConfig c = EnabledConfig();
  c.probe_model = "wavelet";
  AutoExposure ae(c, 500.0, 0.0);
  EXPECT_EQ(ae.config().probe_model, "linear");
}

TEST(ProbeModel, GammaRemainsSelectable)
{
  AutoExposureConfig c = EnabledConfig();
  c.probe_model = "gamma";
  AutoExposure ae(c, 500.0, 0.0);
  EXPECT_EQ(ae.config().probe_model, "gamma");
}

/// The linear probe must model clipping, otherwise brightening a nearly saturated frame looks
/// free and the controller happily drives it into the rail.
TEST(ProbeModel, LinearProbeClipsAtTheRail)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, 500.0, 0.0);
  // A frame already near the top, with enough contrast that the darker probe still clears the
  // gradient threshold. Brightening this can only push it into the rail.
  const std::vector<uint8_t> hot = Textured(215, 40);
  AutoExposureCommand cmd;
  ae.Update(hot.data(), kW, kH, kW, "mono8", 0.0, &cmd);
  EXPECT_LT(cmd.metric_brighter, cmd.metric_darker);
}

// --- Mean-intensity fallback -------------------------------------------------------------------

/// A scene with no gradient above the threshold gives 0/0 for the drive ratio. Rather than
/// freeze, the controller falls back to steering the mean level, and says so.
TEST(MeanFallback, EngagesWhenNoGradientSurvives)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, 500.0, 0.0);
  // Flat mid-grey: no gradient anywhere, but not clipped either, so the saturation guard
  // does not fire and only the fallback can move this.
  const std::vector<uint8_t> blank = Flat(40);
  AutoExposureCommand cmd;

  ASSERT_TRUE(ae.Update(blank.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_TRUE(cmd.mean_fallback);
  EXPECT_FALSE(cmd.saturation_override);
  EXPECT_DOUBLE_EQ(cmd.metric, 0.0);
  // 40/255 is well under the 0.35 target, so it should be brightening.
  EXPECT_GT(cmd.drive, 0.0);
  EXPECT_GT(ae.exposure_us(), 500.0);
}

TEST(MeanFallback, DarkensWhenAboveTarget)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> blank = Flat(200);  // 0.78, well above the 0.35 target
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(blank.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_TRUE(cmd.mean_fallback);
  EXPECT_LT(cmd.drive, 0.0);
  EXPECT_LT(ae.exposure_us(), 500.0);
}

/// The fallback is a last resort: as soon as the scene has structure the metric must drive.
TEST(MeanFallback, YieldsToTheMetricWhenStructureExists)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> textured = Textured(128, 60);
  AutoExposureCommand cmd;
  ae.Update(textured.data(), kW, kH, kW, "mono8", 0.0, &cmd);
  EXPECT_FALSE(cmd.mean_fallback);
  EXPECT_GT(cmd.metric, 0.0);
}

// --- Controller ------------------------------------------------------------------------------

TEST(Controller, DisabledNeverActs)
{
  AutoExposureConfig c = EnabledConfig();
  c.enabled = false;
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> img = Textured(20, 15);
  AutoExposureCommand cmd;
  EXPECT_FALSE(ae.Update(img.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_DOUBLE_EQ(ae.exposure_us(), 500.0);
}

TEST(Controller, DarkSceneRaisesLight)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> dark = Textured(20, 15);
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(dark.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_GT(ae.exposure_us(), 500.0);
}

TEST(Controller, BlownOutSceneLowersLightViaOverride)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  // Almost entirely at the high rail: well past saturation_frac_max.
  std::vector<uint8_t> blown = Flat(255);
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(blown.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_LT(ae.exposure_us(), 500.0);
  EXPECT_GT(cmd.clipped_high_frac, 0.9);
  EXPECT_LT(cmd.drive, 0.0);
}

TEST(Controller, CrushedSceneRaisesLightViaOverride)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  std::vector<uint8_t> crushed = Flat(0);
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_GT(ae.exposure_us(), 500.0);
  EXPECT_GT(cmd.clipped_low_frac, 0.9);
  EXPECT_GT(cmd.drive, 0.0);
}

/// A scene clipping at BOTH rails is beyond what exposure can fix, so the metric must be left
/// in charge rather than one rail arbitrarily winning.
/// The barrier overrides the metric only when they disagree: structure in the unclipped part of
/// the frame says "brighter", the clip budget says "no".
TEST(Controller, BarrierOverridesMetricWhenTheyDisagree)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  // Top 60% blown out, bottom 40% dark but textured, so the metric wants more light.
  std::vector<uint8_t> img = Textured(20, 12);
  for (int y = 0; y < (kH * 6) / 10; ++y)
  {
    for (int x = 0; x < kW; ++x)
    {
      img[static_cast<size_t>(y) * kW + x] = 255;
    }
  }
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(img.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_GT(cmd.metric_brighter, cmd.metric_darker) << "metric should want more light here";
  EXPECT_TRUE(cmd.saturation_override);
  EXPECT_LT(cmd.drive, 0.0);
  EXPECT_LT(ae.exposure_us(), 500.0);
}

/// The barrier's response must scale with how far over budget the frame is, otherwise the loop
/// limit cycles: on the car a fixed full-size cut produced 10.4 -> 12.7 -> 11.6 -> 10.4 dB
/// hunting, which raised the mean metric but lowered the 10th percentile.
TEST(Controller, BarrierResponseIsProportional)
{
  auto clip_fraction = [](double frac) {
    std::vector<uint8_t> img = Textured(120, 50);
    const int rows = static_cast<int>(frac * kH);
    for (int y = 0; y < rows; ++y)
    {
      for (int x = 0; x < kW; ++x)
      {
        img[static_cast<size_t>(y) * kW + x] = 255;
      }
    }
    return img;
  };

  AutoExposureConfig c = EnabledConfig();
  c.saturation_frac_max = 0.02;

  // Just over budget: a small correction.
  AutoExposure near_limit(c, 1000.0, 0.0);
  AutoExposureCommand near_cmd;
  near_limit.Update(clip_fraction(0.025).data(), kW, kH, kW, "mono8", 0.0, &near_cmd);

  // Far over budget: a large one.
  AutoExposure far_over(c, 1000.0, 0.0);
  AutoExposureCommand far_cmd;
  far_over.Update(clip_fraction(0.30).data(), kW, kH, kW, "mono8", 0.0, &far_cmd);

  ASSERT_LT(near_cmd.drive, 0.0);
  ASSERT_LT(far_cmd.drive, 0.0);
  EXPECT_LT(far_cmd.drive, near_cmd.drive) << "further over budget must push back harder";
  // The near-limit correction must be gentle, not a full-size step.
  EXPECT_GT(near_limit.exposure_us(), 0.95 * 1000.0);
  EXPECT_LT(far_over.exposure_us(), near_limit.exposure_us());
}

TEST(Controller, BothRailsClippedFallsBackToMetric)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  std::vector<uint8_t> img = Flat(0);
  for (int y = 0; y < kH / 2; ++y)
  {
    for (int x = 0; x < kW; ++x)
    {
      img[static_cast<size_t>(y) * kW + x] = 255;
    }
  }
  AutoExposureCommand cmd;
  ae.Update(img.data(), kW, kH, kW, "mono8", 0.0, &cmd);
  EXPECT_FALSE(cmd.saturation_override);
  EXPECT_GT(cmd.clipped_low_frac, 0.4);
  EXPECT_GT(cmd.clipped_high_frac, 0.4);
}

TEST(Controller, DeadbandSuppressesWrites)
{
  AutoExposureConfig c = EnabledConfig();
  c.deadband = 1.0;  // nothing can ever exceed this, so no write may be issued
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> img = Textured(128, 60);
  AutoExposureCommand cmd;
  EXPECT_FALSE(ae.Update(img.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_DOUBLE_EQ(ae.exposure_us(), 500.0);
  // Telemetry is still reported even when no write is issued.
  EXPECT_GT(cmd.metric, 0.0);
}

/// Regression: a low-texture scene scores two orders of magnitude below a detailed one, so the
/// RAW difference between the two probes is tiny even when the direction is unambiguous. On the
/// car this showed up as bright=6e-4 / dark=1e-4 -- a clear "go brighter" -- sitting under a
/// 3e-3 absolute deadband and the exposure never moving. The drive signal is normalized so the
/// decision depends on the ratio of the probes, not on the scene's absolute contrast.
TEST(Controller, LowTextureSceneStillDrives)
{
  AutoExposure ae(EnabledConfig(), 493.0, 0.0);
  // Dark and nearly flat: metrics land in the 1e-4 range, as measured on the car.
  const std::vector<uint8_t> dim = LowTexture(25, 20);
  AutoExposureCommand cmd;

  ASSERT_TRUE(ae.Update(dim.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_LT(cmd.metric_brighter, 0.01) << "test scene is not actually low-texture";
  EXPECT_LT(std::fabs(cmd.metric_brighter - cmd.metric_darker), 0.005)
      << "raw difference is not actually small; the regression is not being reproduced";
  // The normalized drive is large even though the raw difference is not.
  EXPECT_GT(cmd.drive, 0.2);
  EXPECT_GT(ae.exposure_us(), 493.0);
}

/// The drive signal is a ratio, so the same scene at a different absolute contrast must produce
/// the same decision rather than a differently sized one.
TEST(Controller, DriveIsScaleFree)
{
  AutoExposure a(EnabledConfig(), 500.0, 0.0);
  AutoExposure b(EnabledConfig(), 500.0, 0.0);
  AutoExposureCommand low, high;
  a.Update(LowTexture(25, 20).data(), kW, kH, kW, "mono8", 0.0, &low);
  b.Update(Textured(60, 40).data(), kW, kH, kW, "mono8", 0.0, &high);
  // Absolute metrics differ substantially between the two...
  EXPECT_GT(high.metric_brighter, 1.5 * low.metric_brighter);
  // ...but both agree the scene wants more light.
  EXPECT_GT(low.drive, 0.0);
  EXPECT_GT(high.drive, 0.0);
}

TEST(Controller, StepIsRateLimited)
{
  AutoExposureConfig c = EnabledConfig();
  c.max_step_ratio = 1.1;
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> crushed = Flat(0);
  AutoExposureCommand cmd;
  ASSERT_TRUE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_LE(ae.exposure_us(), 500.0 * 1.1 + 1e-9);
}

TEST(Controller, UpdateHzLimitsActuationRate)
{
  AutoExposureConfig c = EnabledConfig();
  c.update_hz = 10.0;
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> crushed = Flat(0);
  AutoExposureCommand cmd;

  ASSERT_TRUE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  // 10 ms later: inside the 100 ms period, so declined.
  EXPECT_FALSE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.010, &cmd));
  // 150 ms later: allowed again.
  EXPECT_TRUE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.150, &cmd));
}

/// The actuator policy that matters most on this car: light demand goes to exposure until the
/// blur budget is spent, and only then to gain.
TEST(Controller, ExposureFillsBeforeGain)
{
  AutoExposureConfig c = EnabledConfig();
  c.exposure_min_us = 20.0;
  c.exposure_max_us = 2000.0;
  AutoExposure ae(c, 20.0, 0.0);
  const std::vector<uint8_t> crushed = Flat(0);
  AutoExposureCommand cmd;

  for (int i = 0; i < 40; ++i)
  {
    ae.Update(crushed.data(), kW, kH, kW, "mono8", static_cast<double>(i), &cmd);
    if (ae.exposure_us() < c.exposure_max_us - 1e-6)
    {
      // While there is blur budget left, gain must stay at its floor.
      EXPECT_DOUBLE_EQ(ae.gain_db(), c.gain_min_db) << "gain used at iteration " << i;
    }
  }
  EXPECT_DOUBLE_EQ(ae.exposure_us(), c.exposure_max_us);
  EXPECT_GT(ae.gain_db(), c.gain_min_db);
}

/// And in reverse: brightening gives back gain before it shortens exposure, because gain is the
/// noisy term.
TEST(Controller, GainIsGivenBackBeforeExposure)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, c.exposure_max_us, 12.0);
  const std::vector<uint8_t> blown = Flat(255);
  AutoExposureCommand cmd;

  ASSERT_TRUE(ae.Update(blown.data(), kW, kH, kW, "mono8", 0.0, &cmd));
  EXPECT_LT(ae.gain_db(), 12.0);
  EXPECT_DOUBLE_EQ(ae.exposure_us(), c.exposure_max_us);
}

TEST(Controller, RespectsActuatorLimits)
{
  AutoExposureConfig c = EnabledConfig();
  c.exposure_min_us = 100.0;
  c.exposure_max_us = 200.0;
  c.gain_max_db = 6.0;
  AutoExposure ae(c, 150.0, 0.0);
  AutoExposureCommand cmd;

  const std::vector<uint8_t> crushed = Flat(0);
  for (int i = 0; i < 60; ++i)
  {
    ae.Update(crushed.data(), kW, kH, kW, "mono8", static_cast<double>(i), &cmd);
  }
  EXPECT_LE(ae.exposure_us(), 200.0 + 1e-9);
  EXPECT_LE(ae.gain_db(), 6.0 + 1e-9);

  const std::vector<uint8_t> blown = Flat(255);
  for (int i = 60; i < 160; ++i)
  {
    ae.Update(blown.data(), kW, kH, kW, "mono8", static_cast<double>(i), &cmd);
  }
  EXPECT_GE(ae.exposure_us(), 100.0 - 1e-9);
  EXPECT_GE(ae.gain_db(), 0.0 - 1e-9);
}

/// Saturated in both actuators and still asking for more: nothing changed, so no write.
TEST(Controller, NoWriteWhenAlreadyAtRail)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, c.exposure_max_us, c.gain_max_db);
  const std::vector<uint8_t> crushed = Flat(0);
  AutoExposureCommand cmd;
  EXPECT_FALSE(ae.Update(crushed.data(), kW, kH, kW, "mono8", 0.0, &cmd));
}

TEST(Controller, SyncFromCameraClampsAndAdopts)
{
  AutoExposureConfig c = EnabledConfig();
  c.exposure_max_us = 2000.0;
  AutoExposure ae(c, 500.0, 0.0);
  ae.SyncFromCamera(99999.0, 99.0);
  EXPECT_DOUBLE_EQ(ae.exposure_us(), 2000.0);
  EXPECT_DOUBLE_EQ(ae.gain_db(), c.gain_max_db);
}

// --- Frame handling --------------------------------------------------------------------------

/// Build an RGGB mosaic with independent per-channel levels.
std::vector<uint8_t> Bayer(uint8_t r, uint8_t g, uint8_t b, int w = kW, int h = kH)
{
  std::vector<uint8_t> img(static_cast<size_t>(w) * h);
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      uint8_t v;
      if ((y & 1) == 0)
      {
        v = (x & 1) == 0 ? r : g;
      }
      else
      {
        v = (x & 1) == 0 ? g : b;
      }
      img[static_cast<size_t>(y) * w + x] = v;
    }
  }
  return img;
}

/// Regression: an even stride on a Bayer frame lands on one colour phase forever. Sampling
/// (0,0) of an RGGB mosaic sees only RED, so a saturated BLUE channel is invisible -- and blue
/// is the channel that clips first on this camera (measured B/G = 1.23). The controller was
/// reporting ~1% clipping on a frame that was actually 6% clipped, and pushing well past its
/// own budget as a result.
TEST(FrameHandling, SaturationSeesEveryBayerChannel)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  // Red and green comfortable, blue pinned at the rail.
  const std::vector<uint8_t> img = Bayer(90, 110, 255);
  AutoExposureCommand cmd;
  ae.Update(img.data(), kW, kH, kW, "bayer_rggb8", 0.0, &cmd);
  EXPECT_GT(cmd.clipped_high_frac, 0.9) << "blue saturation must be visible to the guard";
}

/// ...and the converse: a frame whose red channel is at the rail while green and blue are fine
/// must also be caught, so the fix is not just "look at blue instead of red".
TEST(FrameHandling, SaturationCaughtOnAnyChannel)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> img = Bayer(255, 110, 90);
  AutoExposureCommand cmd;
  ae.Update(img.data(), kW, kH, kW, "bayer_rggb8", 0.0, &cmd);
  EXPECT_GT(cmd.clipped_high_frac, 0.9);
}

/// Structure is read from green, the densest phase and the best luminance proxy, so a strong
/// red/blue imbalance must not register as scene texture.
TEST(FrameHandling, StructureReadFromGreenNotTheMosaic)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  // Wildly different R and B, perfectly flat green: there is no real structure here.
  const std::vector<uint8_t> img = Bayer(10, 128, 240);
  AutoExposureCommand cmd;
  ae.Update(img.data(), kW, kH, kW, "bayer_rggb8", 0.0, &cmd);
  EXPECT_DOUBLE_EQ(cmd.metric, 0.0) << "colour mosaic must not be mistaken for texture";
  EXPECT_NEAR(cmd.mean_level, 128.0 / 255.0, 0.01) << "mean level should track green";
}

TEST(FrameHandling, UnknownEncodingIsDeclined)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  const std::vector<uint8_t> img = Textured(20, 15);
  AutoExposureCommand cmd;
  EXPECT_FALSE(ae.Update(img.data(), kW, kH, kW, "mono16", 0.0, &cmd));
  EXPECT_FALSE(ae.Update(img.data(), kW, kH, kW, "32FC1", 0.0, &cmd));
}

TEST(FrameHandling, OddStrideIsCoercedEvenForBayerPhase)
{
  AutoExposureConfig c = EnabledConfig();
  c.sample_stride = 7;
  AutoExposure ae(c, 500.0, 0.0);
  EXPECT_EQ(ae.config().sample_stride % 2, 0);
  EXPECT_EQ(ae.config().sample_stride, 8);
}

TEST(FrameHandling, TinyFrameIsDeclined)
{
  AutoExposureConfig c = EnabledConfig();
  c.sample_stride = 8;
  AutoExposure ae(c, 500.0, 0.0);
  const std::vector<uint8_t> img = Textured(128, 60, 16, 16);
  AutoExposureCommand cmd;
  // 16/8 = 2 samples per side, which leaves no interior for a central difference.
  EXPECT_FALSE(ae.Update(img.data(), 16, 16, 16, "mono8", 0.0, &cmd));
}

TEST(FrameHandling, NullDataIsDeclined)
{
  AutoExposure ae(EnabledConfig(), 500.0, 0.0);
  AutoExposureCommand cmd;
  EXPECT_FALSE(ae.Update(nullptr, kW, kH, kW, "mono8", 0.0, &cmd));
}

/// A padded row stride must be honoured, otherwise the sampler shears the image and manufactures
/// gradients that are not in the scene.
TEST(FrameHandling, RowPaddingIsHonoured)
{
  AutoExposureConfig c = EnabledConfig();
  AutoExposure ae(c, 500.0, 0.0);

  const uint32_t pad = 37;
  std::vector<uint8_t> padded(static_cast<size_t>(kH) * (kW + pad), 0);
  const std::vector<uint8_t> tight = Textured(128, 60);
  for (int y = 0; y < kH; ++y)
  {
    std::copy(tight.begin() + static_cast<size_t>(y) * kW,
              tight.begin() + static_cast<size_t>(y + 1) * kW,
              padded.begin() + static_cast<size_t>(y) * (kW + pad));
  }

  AutoExposureCommand padded_cmd;
  ae.Update(padded.data(), kW, kH, kW + pad, "mono8", 0.0, &padded_cmd);

  AutoExposure ae_tight(c, 500.0, 0.0);
  AutoExposureCommand tight_cmd;
  ae_tight.Update(tight.data(), kW, kH, kW, "mono8", 0.0, &tight_cmd);

  EXPECT_NEAR(padded_cmd.metric, tight_cmd.metric, 1e-12);
}

/// Interleaved colour: the sampler reads the first byte of each pixel, so a 3-byte-per-pixel
/// frame must produce the same statistic as the mono frame built from those same first bytes.
TEST(FrameHandling, InterleavedColourUsesFirstChannel)
{
  AutoExposureConfig c = EnabledConfig();
  const std::vector<uint8_t> mono = Textured(128, 60);
  std::vector<uint8_t> rgb(static_cast<size_t>(kW) * kH * 3, 0);
  for (size_t i = 0; i < mono.size(); ++i)
  {
    rgb[i * 3 + 0] = mono[i];
    rgb[i * 3 + 1] = 17;  // deliberately unrelated
    rgb[i * 3 + 2] = 200;
  }

  AutoExposure ae_rgb(c, 500.0, 0.0);
  AutoExposureCommand rgb_cmd;
  ae_rgb.Update(rgb.data(), kW, kH, kW * 3, "rgb8", 0.0, &rgb_cmd);

  AutoExposure ae_mono(c, 500.0, 0.0);
  AutoExposureCommand mono_cmd;
  ae_mono.Update(mono.data(), kW, kH, kW, "mono8", 0.0, &mono_cmd);

  EXPECT_NEAR(rgb_cmd.metric, mono_cmd.metric, 1e-12);
}

}  // namespace avt_vimba_camera
