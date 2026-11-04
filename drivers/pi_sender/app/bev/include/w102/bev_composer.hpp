#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace w102 {

// Draws a metric vehicle footprint at the centre of the BEV. The provisional
// camera geometry deliberately leaves this footprint unobserved; drawing it
// last prevents camera pixels from pretending to show the vehicle body.
void draw_vehicle_overlay(cv::Mat& bev_bgr, double vehicle_length_m,
                          double vehicle_width_m, double meters_per_pixel);

class BevCameraTransform {
 public:
  BevCameraTransform() = default;
  void load(const std::string& calibration_path, const cv::Size& input_size,
            const cv::Size& bev_size);
  bool ready() const;
  void project(const cv::Mat& input_bgr, cv::Mat& projected_bgr) const;
  const cv::Mat& weight() const;

 private:
  bool ready_{false};
  cv::Size input_size_{};
  cv::Size bev_size_{};
  cv::Mat direct_map_1_;
  cv::Mat direct_map_2_;
  cv::Mat homography_;
  cv::Mat weight_;
};

class BevComposer {
 public:
  explicit BevComposer(cv::Size output_size);
  void add_camera(const std::string& calibration_path,
                  const cv::Size& input_size);
  bool ready(std::size_t expected_camera_count) const;
  // Precomputes normalized blend weights for every possible active-camera
  // mask and the structural corner-fill map. Calling this before the worker
  // starts keeps all geometry and fusion setup out of the real-time path.
  void prepare() const;
  cv::Mat compose(const std::vector<cv::Mat>& frames,
                  cv::Mat* coverage_mask,
                  bool fill_uncovered = true,
                  bool* synthetic_fill_applied = nullptr,
                  bool soften_fill = true) const;

 private:
  struct BlendPlan {
    cv::Mat valid;
    std::vector<cv::Mat> normalized_weights;
  };

  cv::Size output_size_;
  std::vector<BevCameraTransform> transforms_;
  mutable bool blend_plans_ready_{false};
  mutable std::array<BlendPlan, 16> blend_plans_{};
  mutable bool fill_maps_ready_{false};
  mutable cv::Mat fill_map_x_;
  mutable cv::Mat fill_map_y_;
  mutable cv::Mat full_coverage_;
};

}  // namespace w102
