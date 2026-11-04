#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace w102 {

struct IlluminationStats {
  double input_p01{0.0};
  double input_median{0.0};
  double input_p99{0.0};
  double shadow_fraction{0.0};
  double highlight_fraction{0.0};
  double applied_gamma{1.0};
};

class AdaptiveIllumination {
 public:
  explicit AdaptiveIllumination(std::size_t camera_count);

  cv::Mat enhance(const cv::Mat& input_bgr, std::size_t camera_index,
                  IlluminationStats* stats = nullptr,
                  bool economical = false);
  static void enhance_local_detail(cv::Mat& input_output_bgr,
                                   bool low_light);

 private:
  struct TemporalState {
    bool initialized{false};
    double low{0.0};
    double median{110.0};
    double high{255.0};
  };

  std::vector<TemporalState> states_;
};

}  // namespace w102
