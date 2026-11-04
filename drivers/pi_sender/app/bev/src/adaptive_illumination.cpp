#include "w102/adaptive_illumination.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace w102 {
namespace {

struct LuminanceAnalysis {
  double p01{0.0};
  double median{0.0};
  double p99{255.0};
  double shadows{0.0};
  double highlights{0.0};
};

LuminanceAnalysis analyze(const cv::Mat& luminance) {
  cv::Mat sample;
  const double scale = std::min(1.0, 320.0 / luminance.cols);
  if (scale < 1.0) {
    cv::resize(luminance, sample, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    sample = luminance;
  }
  std::array<std::uint32_t, 256> histogram{};
  for (int row = 0; row < sample.rows; ++row) {
    const auto* values = sample.ptr<std::uint8_t>(row);
    for (int column = 0; column < sample.cols; ++column) {
      ++histogram[values[column]];
    }
  }
  const std::uint64_t total = static_cast<std::uint64_t>(sample.total());
  auto percentile = [&](double fraction) {
    const auto target = static_cast<std::uint64_t>(fraction * total);
    std::uint64_t cumulative = 0;
    for (std::size_t value = 0; value < histogram.size(); ++value) {
      cumulative += histogram[value];
      if (cumulative >= target) return static_cast<double>(value);
    }
    return 255.0;
  };
  LuminanceAnalysis result;
  result.p01 = percentile(0.01);
  result.median = percentile(0.50);
  result.p99 = percentile(0.99);
  std::uint64_t shadows = 0;
  std::uint64_t highlights = 0;
  for (int value = 0; value <= 15; ++value) shadows += histogram[value];
  for (int value = 245; value <= 255; ++value) highlights += histogram[value];
  result.shadows = static_cast<double>(shadows) / total;
  result.highlights = static_cast<double>(highlights) / total;
  return result;
}

}  // namespace

AdaptiveIllumination::AdaptiveIllumination(std::size_t camera_count)
    : states_(camera_count) {}

cv::Mat AdaptiveIllumination::enhance(const cv::Mat& input_bgr,
                                      std::size_t camera_index,
                                      IlluminationStats* stats,
                                      bool economical) {
  if (input_bgr.empty() || input_bgr.type() != CV_8UC3 ||
      camera_index >= states_.size()) {
    throw std::invalid_argument("invalid illumination input or camera index");
  }
  const double scale = std::min(1.0, 320.0 / input_bgr.cols);
  cv::Mat sample_bgr;
  if (scale < 1.0) {
    cv::resize(input_bgr, sample_bgr, cv::Size(), scale, scale,
               cv::INTER_AREA);
  } else {
    sample_bgr = input_bgr;
  }
  cv::Mat sample_luminance;
  cv::cvtColor(sample_bgr, sample_luminance, cv::COLOR_BGR2GRAY);
  const auto analysis = analyze(sample_luminance);

  auto& state = states_[camera_index];
  constexpr double alpha = 0.10;  // avoids visible pumping at seams
  if (!state.initialized) {
    state.low = analysis.p01;
    state.median = analysis.median;
    state.high = analysis.p99;
    state.initialized = true;
  } else {
    state.low += alpha * (analysis.p01 - state.low);
    state.median += alpha * (analysis.median - state.median);
    state.high += alpha * (analysis.p99 - state.high);
  }

  // Every camera converges toward a common midtone. This reduces left/right
  // seam brightness while bounded gamma avoids turning night noise into detail.
  const double normalized_median = std::clamp(
      (state.median - state.low) / std::max(16.0, state.high - state.low),
      0.03, 0.97);
  const double target_median = 0.45;
  const double gamma = std::clamp(
      std::log(target_median) / std::log(normalized_median), 0.55, 1.80);
  std::array<std::uint8_t, 256> values{};
  const double low = std::clamp(state.low - 3.0, 0.0, 220.0);
  const double high = std::clamp(state.high + 3.0, low + 24.0, 255.0);
  for (std::size_t index = 0; index < values.size(); ++index) {
    const double normalized =
        std::clamp((static_cast<double>(index) - low) / (high - low), 0.0, 1.0);
    // Smooth shoulder/toe compresses strong sunlight highlights and opens
    // shadows before bounded gamma correction.
    const double filmic = normalized * normalized * (3.0 - 2.0 * normalized);
    values[index] = static_cast<std::uint8_t>(
        std::lround(255.0 * std::pow(filmic, gamma)));
  }
  cv::Mat lut(1, 256, CV_8UC1, values.data());
  cv::Mat output;
  cv::LUT(input_bgr, lut, output);

  // CLAHE can push a nearly saturated scene back toward clipping. Apply a
  // bounded global shoulder based on the measured clipped-pixel fraction. The
  // small-obstacle local contrast is retained while headroom is restored.
  if (analysis.highlights > 0.02) {
    const double highlight_gain = 1.0 -
        std::min(0.30, (analysis.highlights - 0.02) * 0.40);
    output.convertTo(output, -1, highlight_gain, 0.0);
  }

  // Mild unsharp masking enhances obstacle edges without the cost of a neural
  // low-light model or amplifying high-frequency noise aggressively.
  if (!economical) enhance_local_detail(output, analysis.median < 35.0);

  if (stats != nullptr) {
    stats->input_p01 = analysis.p01;
    stats->input_median = analysis.median;
    stats->input_p99 = analysis.p99;
    stats->shadow_fraction = analysis.shadows;
    stats->highlight_fraction = analysis.highlights;
    stats->applied_gamma = gamma;
  }
  return output;
}

void AdaptiveIllumination::enhance_local_detail(cv::Mat& input_output_bgr,
                                                bool low_light) {
  if (input_output_bgr.empty() || input_output_bgr.type() != CV_8UC3) return;
  cv::Mat lab;
  cv::cvtColor(input_output_bgr, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);
  auto clahe = cv::createCLAHE(low_light ? 1.25 : 1.8, cv::Size(8, 8));
  clahe->apply(channels[0], channels[0]);
  cv::merge(channels, lab);
  cv::cvtColor(lab, input_output_bgr, cv::COLOR_Lab2BGR);
  if (!low_light) {
    cv::Mat blurred;
    cv::GaussianBlur(input_output_bgr, blurred, cv::Size(0, 0), 1.0);
    cv::addWeighted(input_output_bgr, 1.10, blurred, -0.10, 0.0,
                    input_output_bgr);
  }
}

}  // namespace w102
