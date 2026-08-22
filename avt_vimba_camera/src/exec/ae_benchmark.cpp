/// Copyright (c) 2026, AI Racing Tech
///
/// Scores a live camera stream with the gradient metric, independently of whatever is choosing
/// the exposure. Run it once against the camera's own AE and once against the in-driver
/// controller to compare them on the same yardstick: the controller's own telemetry cannot be
/// used for that, since it would be marking its own homework.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "avt_vimba_camera/auto_exposure.hpp"

namespace
{

double Percentile(std::vector<double> v, double p)
{
  if (v.empty())
  {
    return 0.0;
  }
  std::sort(v.begin(), v.end());
  const size_t i = static_cast<size_t>(
      std::min<double>(v.size() - 1, std::max(0.0, p * (v.size() - 1))));
  return v[i];
}

double Mean(const std::vector<double>& v)
{
  if (v.empty())
  {
    return 0.0;
  }
  double s = 0.0;
  for (double x : v)
  {
    s += x;
  }
  return s / v.size();
}

}  // namespace

class AeBenchmark : public rclcpp::Node
{
public:
  AeBenchmark() : Node("ae_benchmark")
  {
    topic_ = declare_parameter("topic", std::string("/vimba_front_left_center/image"));
    duration_s_ = declare_parameter("duration_s", 30.0);
    label_ = declare_parameter("label", std::string("run"));
    // Optional: dump the first frame verbatim so the exposure surrogate can be validated
    // offline against the measured sweep instead of being assumed.
    dump_path_ = declare_parameter("dump_path", std::string(""));

    avt_vimba_camera::AutoExposureConfig cfg;
    cfg.enabled = true;
    cfg.update_hz = 0.0;   // score every frame that arrives
    cfg.deadband = 1e9;    // never act; this instance only ever measures
    cfg.sample_stride = static_cast<int>(declare_parameter("sample_stride", int64_t{ 8 }));
    scorer_ = std::unique_ptr<avt_vimba_camera::AutoExposure>(
        new avt_vimba_camera::AutoExposure(cfg, 1000.0, 0.0));

    sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnImage(msg); });

    start_ = now();
    RCLCPP_INFO(get_logger(), "[%s] scoring %s for %.0f s", label_.c_str(), topic_.c_str(),
                duration_s_);
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { Tick(); });
  }

private:
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    if (!dump_path_.empty() && !dumped_)
    {
      dumped_ = true;
      FILE* f = std::fopen(dump_path_.c_str(), "wb");
      if (f != nullptr)
      {
        std::fprintf(f, "%u %u %u %s\n", msg->width, msg->height, msg->step,
                     msg->encoding.c_str());
        std::fwrite(msg->data.data(), 1, msg->data.size(), f);
        std::fclose(f);
        RCLCPP_INFO(get_logger(), "dumped frame to %s", dump_path_.c_str());
      }
    }

    avt_vimba_camera::AutoExposureCommand cmd;
    scorer_->Update(msg->data.data(), msg->width, msg->height, msg->step, msg->encoding,
                    rclcpp::Time(msg->header.stamp).seconds(), &cmd);
    if (cmd.metric <= 0.0 && cmd.clipped_low_frac == 0.0 && cmd.clipped_high_frac == 0.0)
    {
      return;  // frame declined (unsupported encoding or geometry)
    }
    metric_.push_back(cmd.metric);
    level_.push_back(cmd.mean_level);
    clip_low_.push_back(cmd.clipped_low_frac);
    clip_high_.push_back(cmd.clipped_high_frac);
  }

  void Tick()
  {
    if ((now() - start_).seconds() < duration_s_)
    {
      return;
    }
    Report();
    rclcpp::shutdown();
  }

  void Report()
  {
    const double elapsed = (now() - start_).seconds();
    std::printf("\n=== %s ===\n", label_.c_str());
    std::printf("topic            %s\n", topic_.c_str());
    std::printf("frames scored    %zu over %.1f s (%.1f Hz)\n", metric_.size(), elapsed,
                metric_.empty() ? 0.0 : metric_.size() / elapsed);
    if (metric_.empty())
    {
      std::printf("NO FRAMES -- nothing published on this topic\n");
      return;
    }
    std::printf("metric mean      %.5f\n", Mean(metric_));
    std::printf("metric p10       %.5f   (worst-case frames: what a detector sees at its worst)\n",
                Percentile(metric_, 0.10));
    std::printf("metric p50       %.5f\n", Percentile(metric_, 0.50));
    std::printf("metric p90       %.5f\n", Percentile(metric_, 0.90));
    // Frame-to-frame brightness swing. This is what "blinking" actually is: a stable stream
    // has p95-p5 of a fraction of a percent, mains flicker or a hunting AE shows several.
    const double l5 = Percentile(level_, 0.05), l50 = Percentile(level_, 0.50),
                 l95 = Percentile(level_, 0.95);
    std::printf("mean level       p5 %.4f  p50 %.4f  p95 %.4f\n", l5, l50, l95);
    std::printf("BRIGHTNESS SWING %.2f%% of p50   <-- blinking indicator\n",
                l50 > 1e-9 ? 100.0 * (l95 - l5) / l50 : 0.0);
    std::printf("clipped low      %.2f%% mean, %.2f%% p90\n", 100.0 * Mean(clip_low_),
                100.0 * Percentile(clip_low_, 0.90));
    std::printf("clipped high     %.2f%% mean, %.2f%% p90\n", 100.0 * Mean(clip_high_),
                100.0 * Percentile(clip_high_, 0.90));
    std::fflush(stdout);
  }

  std::string topic_;
  std::string label_;
  std::string dump_path_;
  bool dumped_{ false };
  double duration_s_{ 30.0 };
  rclcpp::Time start_;
  std::unique_ptr<avt_vimba_camera::AutoExposure> scorer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<double> metric_;
  std::vector<double> clip_low_;
  std::vector<double> level_;
  std::vector<double> clip_high_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AeBenchmark>());
  rclcpp::shutdown();
  return 0;
}
