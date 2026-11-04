#include "w102/bev_composer.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <future>
#include <stdexcept>

namespace w102 {

void draw_vehicle_overlay(cv::Mat& bev_bgr, double vehicle_length_m,
                          double vehicle_width_m, double meters_per_pixel) {
  if (bev_bgr.empty() || bev_bgr.type() != CV_8UC3 ||
      vehicle_length_m <= 0.0 || vehicle_width_m <= 0.0 ||
      meters_per_pixel <= 0.0) {
    throw std::invalid_argument("invalid vehicle overlay parameters");
  }
  const int half_length = std::max(
      3, static_cast<int>(std::lround(vehicle_length_m /
                                     (2.0 * meters_per_pixel))));
  const int half_width = std::max(
      3, static_cast<int>(std::lround(vehicle_width_m /
                                     (2.0 * meters_per_pixel))));
  if (2 * half_length >= bev_bgr.rows || 2 * half_width >= bev_bgr.cols) {
    throw std::invalid_argument("vehicle overlay does not fit BEV canvas");
  }
  const cv::Point centre(bev_bgr.cols / 2, bev_bgr.rows / 2);
  const int nose_inset = std::max(2, half_width / 3);
  const int chamfer = std::max(3, half_length / 8);
  std::vector<cv::Point> body{
      {centre.x - half_width + nose_inset, centre.y - half_length},
      {centre.x + half_width - nose_inset, centre.y - half_length},
      {centre.x + half_width, centre.y - half_length + chamfer},
      {centre.x + half_width, centre.y + half_length - chamfer},
      {centre.x + half_width - nose_inset, centre.y + half_length},
      {centre.x - half_width + nose_inset, centre.y + half_length},
      {centre.x - half_width, centre.y + half_length - chamfer},
      {centre.x - half_width, centre.y - half_length + chamfer}};
  cv::fillConvexPoly(bev_bgr, body, cv::Scalar(38, 42, 46), cv::LINE_AA);
  cv::polylines(bev_bgr, body, true, cv::Scalar(225, 225, 225), 2,
                cv::LINE_AA);

  // Arrow points toward the provisional vehicle front (top of the BEV).
  const int arrow_length = std::max(10, half_length / 2);
  cv::arrowedLine(bev_bgr, centre,
                  cv::Point(centre.x, centre.y - arrow_length),
                  cv::Scalar(80, 220, 80), 3, cv::LINE_AA, 0, 0.30);
}

void BevCameraTransform::load(const std::string& calibration_path,
                              const cv::Size& input_size,
                              const cv::Size& bev_size) {
  cv::FileStorage storage(calibration_path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("cannot open calibration: " + calibration_path);
  }
  cv::Mat intrinsic;
  cv::Mat distortion;
  storage["K"] >> intrinsic;
  storage["D"] >> distortion;
  storage["H_bev"] >> homography_;
  int calibrated_width = 0;
  int calibrated_height = 0;
  int calibrated_bev_width = 0;
  int calibrated_bev_height = 0;
  storage["image_width"] >> calibrated_width;
  storage["image_height"] >> calibrated_height;
  storage["bev_width"] >> calibrated_bev_width;
  storage["bev_height"] >> calibrated_bev_height;
  std::string remap_file;
  storage["bev_remap_file"] >> remap_file;
  if (intrinsic.empty() || distortion.empty() || homography_.empty() ||
      calibrated_width != input_size.width ||
      calibrated_height != input_size.height) {
    throw std::runtime_error(
        "calibration matrices missing or resolution mismatch: " +
        calibration_path);
  }

  input_size_ = input_size;
  bev_size_ = bev_size;
  // Existing calibrations without explicit BEV dimensions retain their old
  // behaviour. New calibrations declare the canvas used to produce H_bev so
  // one calibration can be rendered at 768, 640 or 512 without cropping.
  if (calibrated_bev_width <= 0) calibrated_bev_width = bev_size.width;
  if (calibrated_bev_height <= 0) calibrated_bev_height = bev_size.height;
  cv::Mat scale = cv::Mat::eye(3, 3, CV_64F);
  scale.at<double>(0, 0) =
      static_cast<double>(bev_size.width) / calibrated_bev_width;
  scale.at<double>(1, 1) =
      static_cast<double>(bev_size.height) / calibrated_bev_height;
  homography_.convertTo(homography_, CV_64F);
  homography_ = scale * homography_;

  // Reviewed calibrations may contain an exact raw-fisheye -> BEV lookup.
  // Prefer it over reconstructing a map from K/D/H: H_bev is defined on the
  // rectified image and cannot be applied directly to equidistant raw pixels.
  if (!remap_file.empty()) {
    if (calibrated_bev_width != bev_size.width ||
        calibrated_bev_height != bev_size.height) {
      throw std::runtime_error(
          "precomputed BEV remap resolution does not match runtime canvas: " +
          calibration_path);
    }
    std::filesystem::path map_path(remap_file);
    if (map_path.is_relative()) {
      map_path = std::filesystem::path(calibration_path).parent_path() / map_path;
    }
    cv::FileStorage map_storage(map_path.string(), cv::FileStorage::READ);
    if (!map_storage.isOpened()) {
      throw std::runtime_error("cannot open precomputed BEV remap: " +
                               map_path.string());
    }
    cv::Mat direct_map_x;
    cv::Mat direct_map_y;
    map_storage["map_x"] >> direct_map_x;
    map_storage["map_y"] >> direct_map_y;
    if (direct_map_x.empty() || direct_map_y.empty() ||
        direct_map_x.size() != bev_size_ || direct_map_y.size() != bev_size_ ||
        direct_map_x.channels() != 1 || direct_map_y.channels() != 1 ||
        !cv::checkRange(direct_map_x) || !cv::checkRange(direct_map_y)) {
      throw std::runtime_error("invalid precomputed BEV remap: " +
                               map_path.string());
    }
    direct_map_x.convertTo(direct_map_x, CV_32FC1);
    direct_map_y.convertTo(direct_map_y, CV_32FC1);

    cv::Mat x_min;
    cv::Mat x_max;
    cv::Mat y_min;
    cv::Mat y_max;
    cv::compare(direct_map_x, 0.0, x_min, cv::CMP_GE);
    cv::compare(direct_map_x, static_cast<double>(input_size_.width - 1),
                x_max, cv::CMP_LT);
    cv::compare(direct_map_y, 0.0, y_min, cv::CMP_GE);
    cv::compare(direct_map_y, static_cast<double>(input_size_.height - 1),
                y_max, cv::CMP_LT);
    cv::bitwise_and(x_min, x_max, weight_);
    cv::bitwise_and(weight_, y_min, weight_);
    cv::bitwise_and(weight_, y_max, weight_);
    cv::erode(weight_, weight_,
              cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::distanceTransform(weight_, weight_, cv::DIST_L2, 3);
    cv::min(weight_, 64.0, weight_);
    weight_ /= 64.0;
    cv::convertMaps(direct_map_x, direct_map_y, direct_map_1_, direct_map_2_,
                    CV_16SC2);
    ready_ = true;
    return;
  }

  cv::Mat map_x;
  cv::Mat map_y;
  cv::fisheye::initUndistortRectifyMap(
      intrinsic, distortion, cv::Mat::eye(3, 3, CV_64F), intrinsic,
      input_size_, CV_32FC1, map_x, map_y);

  // Compose undistortion and ground projection once. Runtime projection is a
  // single fixed-point remap rather than remap + warpPerspective.
  cv::Mat direct_map_x;
  cv::Mat direct_map_y;
  cv::warpPerspective(map_x, direct_map_x, homography_, bev_size_,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1));
  cv::warpPerspective(map_y, direct_map_y, homography_, bev_size_,
                      cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(-1));
  cv::convertMaps(direct_map_x, direct_map_y, direct_map_1_, direct_map_2_,
                  CV_16SC2);

  cv::Mat valid_source(input_size_, CV_8UC1, cv::Scalar(255));
  cv::Mat valid_undistorted;
  cv::remap(valid_source, valid_undistorted, map_x, map_y, cv::INTER_NEAREST,
            cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::Mat valid_bev;
  cv::warpPerspective(valid_undistorted, valid_bev, homography_, bev_size_,
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::threshold(valid_bev, valid_bev, 127, 255, cv::THRESH_BINARY);
  cv::erode(valid_bev, valid_bev,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::distanceTransform(valid_bev, weight_, cv::DIST_L2, 3);
  cv::min(weight_, 64.0, weight_);
  weight_ /= 64.0;
  ready_ = true;
}

bool BevCameraTransform::ready() const { return ready_; }

const cv::Mat& BevCameraTransform::weight() const { return weight_; }

void BevCameraTransform::project(const cv::Mat& input_bgr,
                                 cv::Mat& projected_bgr) const {
  if (!ready_ || input_bgr.size() != input_size_) {
    throw std::runtime_error("BEV camera transform is not ready");
  }
  cv::remap(input_bgr, projected_bgr, direct_map_1_, direct_map_2_,
            cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

BevComposer::BevComposer(cv::Size output_size) : output_size_(output_size) {}

void BevComposer::add_camera(const std::string& calibration_path,
                             const cv::Size& input_size) {
  BevCameraTransform transform;
  transform.load(calibration_path, input_size, output_size_);
  transforms_.push_back(std::move(transform));
  blend_plans_ready_ = false;
  for (auto& plan : blend_plans_) {
    plan.valid.release();
    plan.normalized_weights.clear();
  }
  fill_maps_ready_ = false;
  fill_map_x_.release();
  fill_map_y_.release();
  full_coverage_.release();
}

bool BevComposer::ready(std::size_t expected_camera_count) const {
  if (expected_camera_count == 0 || transforms_.size() != expected_camera_count) {
    return false;
  }
  for (const auto& transform : transforms_) {
    if (!transform.ready()) {
      return false;
    }
  }
  return true;
}

void BevComposer::prepare() const {
  if (blend_plans_ready_) return;
  if (transforms_.empty() || transforms_.size() > 4) {
    throw std::runtime_error("BEV composer supports one to four cameras");
  }

  const std::uint16_t plan_count =
      static_cast<std::uint16_t>(1U << transforms_.size());
  for (std::uint16_t mask = 0; mask < plan_count; ++mask) {
    auto& plan = blend_plans_[mask];
    plan.normalized_weights.assign(transforms_.size(), cv::Mat());
    cv::Mat weight_sum(output_size_, CV_32FC1, cv::Scalar(0));
    for (std::size_t index = 0; index < transforms_.size(); ++index) {
      if ((mask & (1U << index)) != 0) weight_sum += transforms_[index].weight();
    }
    cv::compare(weight_sum, 1e-4, plan.valid, cv::CMP_GT);
    cv::Mat safe_weight;
    cv::max(weight_sum, 1e-6, safe_weight);
    for (std::size_t index = 0; index < transforms_.size(); ++index) {
      if ((mask & (1U << index)) != 0) {
        cv::divide(transforms_[index].weight(), safe_weight,
                   plan.normalized_weights[index]);
      }
    }
  }

  const auto full_mask = static_cast<std::uint16_t>(plan_count - 1U);
  full_coverage_ = blend_plans_[full_mask].valid.clone();
  cv::Mat structural_invalid;
  cv::bitwise_not(full_coverage_, structural_invalid);
  cv::Mat distance;
  cv::Mat labels;
  cv::distanceTransform(structural_invalid, distance, labels, cv::DIST_L2,
                        cv::DIST_MASK_5, cv::DIST_LABEL_PIXEL);
  double maximum_label = 0.0;
  cv::minMaxLoc(labels, nullptr, &maximum_label);
  std::vector<cv::Point2f> label_coordinates(
      static_cast<std::size_t>(maximum_label) + 1U,
      cv::Point2f(-1.0F, -1.0F));
  for (int row = 0; row < full_coverage_.rows; ++row) {
    const auto* full_row = full_coverage_.ptr<std::uint8_t>(row);
    const auto* label_row = labels.ptr<int>(row);
    for (int column = 0; column < full_coverage_.cols; ++column) {
      const int label = label_row[column];
      if (full_row[column] != 0 && label > 0) {
        label_coordinates[static_cast<std::size_t>(label)] =
            cv::Point2f(static_cast<float>(column), static_cast<float>(row));
      }
    }
  }
  fill_map_x_.create(full_coverage_.size(), CV_32FC1);
  fill_map_y_.create(full_coverage_.size(), CV_32FC1);
  for (int row = 0; row < full_coverage_.rows; ++row) {
    const auto* full_row = full_coverage_.ptr<std::uint8_t>(row);
    const auto* label_row = labels.ptr<int>(row);
    auto* map_x_row = fill_map_x_.ptr<float>(row);
    auto* map_y_row = fill_map_y_.ptr<float>(row);
    for (int column = 0; column < full_coverage_.cols; ++column) {
      cv::Point2f source(static_cast<float>(column), static_cast<float>(row));
      if (full_row[column] == 0 && label_row[column] > 0) {
        source = label_coordinates[
            static_cast<std::size_t>(label_row[column])];
      }
      map_x_row[column] = source.x;
      map_y_row[column] = source.y;
    }
  }
  fill_maps_ready_ = true;
  blend_plans_ready_ = true;
}

cv::Mat BevComposer::compose(const std::vector<cv::Mat>& frames,
                             cv::Mat* coverage_mask,
                             bool fill_uncovered,
                             bool* synthetic_fill_applied,
                             bool soften_fill) const {
  if (!ready(frames.size())) {
    throw std::runtime_error("BEV composer does not match supplied frames");
  }
  prepare();

  std::uint16_t active_mask = 0;
  std::array<cv::Mat, 4> projected;
  std::array<std::future<cv::Mat>, 4> projection_tasks;
  for (std::size_t index = 0; index < frames.size(); ++index) {
    if (frames[index].empty()) continue;
    active_mask |= static_cast<std::uint16_t>(1U << index);
    projection_tasks[index] = std::async(
        std::launch::async, [this, &frames, index]() {
          cv::Mat image;
          transforms_[index].project(frames[index], image);
          return image;
        });
  }
  for (std::size_t index = 0; index < frames.size(); ++index) {
    if (projection_tasks[index].valid()) {
      projected[index] = projection_tasks[index].get();
    }
  }

  const auto& plan = blend_plans_[active_mask];
  cv::Mat output(output_size_, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::parallel_for_(cv::Range(0, output.rows), [&](const cv::Range& range) {
    for (int row = range.start; row < range.end; ++row) {
      auto* output_row = output.ptr<cv::Vec3b>(row);
      std::array<const cv::Vec3b*, 4> image_rows{};
      std::array<const float*, 4> weight_rows{};
      for (std::size_t index = 0; index < frames.size(); ++index) {
        if ((active_mask & (1U << index)) == 0) continue;
        image_rows[index] = projected[index].ptr<cv::Vec3b>(row);
        weight_rows[index] =
            plan.normalized_weights[index].ptr<float>(row);
      }
      for (int column = 0; column < output.cols; ++column) {
        cv::Vec3f sum(0.0F, 0.0F, 0.0F);
        for (std::size_t index = 0; index < frames.size(); ++index) {
          if ((active_mask & (1U << index)) == 0) continue;
          const float weight = weight_rows[index][column];
          const auto& pixel = image_rows[index][column];
          sum[0] += weight * pixel[0];
          sum[1] += weight * pixel[1];
          sum[2] += weight * pixel[2];
        }
        output_row[column] = cv::Vec3b(
            cv::saturate_cast<std::uint8_t>(sum[0]),
            cv::saturate_cast<std::uint8_t>(sum[1]),
            cv::saturate_cast<std::uint8_t>(sum[2]));
      }
    }
  });
  bool filled = false;
  if (fill_uncovered) {
    // distanceTransformWithLabels assigns each uncovered pixel to its nearest
    // valid source pixel. This extends adjacent views into unobserved corners,
    // avoiding black star-shaped wedges. The original valid mask is retained so
    // downstream consumers can distinguish measured and synthetic pixels.
    cv::Mat structural_invalid;
    cv::bitwise_not(full_coverage_, structural_invalid);
    filled = cv::countNonZero(structural_invalid) > 0;
    cv::Mat extended;
    cv::remap(output, extended, fill_map_x_, fill_map_y_, cv::INTER_NEAREST,
              cv::BORDER_REPLICATE);
    if (soften_fill) {
      cv::Mat softened;
      cv::GaussianBlur(extended, softened, cv::Size(0, 0), 5.0);
      softened.copyTo(output, structural_invalid);
    } else {
      extended.copyTo(output, structural_invalid);
    }
  }
  if (coverage_mask != nullptr) {
    *coverage_mask = plan.valid;
  }
  if (synthetic_fill_applied != nullptr) {
    *synthetic_fill_applied = filled;
  }
  return output;
}

}  // namespace w102
