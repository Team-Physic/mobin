// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

// YOLO person Instance Label Map과 동기화된 LiDAR point를 결합해 사람의 3D 위치를 계산한다.
// PointCloud timestamp의 TF로 동일 pixel 깊이를 비교하고 PCL RANSAC으로 바닥을 제거한다.
// 최근접 depth candidate의 point 지지가 충분한 관측만 odom 기준 Detection3D로 발행한다.
// Ground/non-ground cloud, plane marker, diagnostics와 선택적 Camera debug image를 함께 발행한다.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"
#include "mobile_robot_lab_cpp/ground_plane_filter.hpp"

namespace mobile_robot_lab_cpp
{

using LabelDetectionCloudPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray,
  sensor_msgs::msg::PointCloud2>;
using ImageLabelDetectionCloudPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, sensor_msgs::msg::Image,
  vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2>;

struct TransformParts
{
  Rotation3 rotation;
  Point3 translation;
};

// tf2 quaternion transform을 point 계산용 3×3 회전행렬과 meter 이동량으로 푼다.
TransformParts transform_parts(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & quaternion = transform.transform.rotation;
  tf2::Matrix3x3 matrix(tf2::Quaternion(
      quaternion.x, quaternion.y, quaternion.z, quaternion.w));
  Rotation3 rotation{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation[static_cast<std::size_t>(row)][static_cast<std::size_t>(column)] =
        matrix[row][column];
    }
  }
  const auto & translation = transform.transform.translation;
  return {rotation, {translation.x, translation.y, translation.z}};
}

sensor_msgs::msg::PointCloud2 point_cloud(
  const std_msgs::msg::Header & header, const std::vector<Point3> & points)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = header;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  for (const auto & point : points) {
    *x = static_cast<float>(point.x);
    *y = static_cast<float>(point.y);
    *z = static_cast<float>(point.z);
    ++x;
    ++y;
    ++z;
  }
  return output;
}

class LidarBboxAssociationNode : public rclcpp::Node
{
public:
  LidarBboxAssociationNode()
  : Node("lidar_bbox_association_cpp"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    points_topic_ = declare_parameter("points_topic", "/calib/points");
    camera_info_topic_ = declare_parameter("camera_info_topic", "/camera/camera_info");
    detections_topic_ = declare_parameter("detections_topic", "/detections");
    labels_topic_ = declare_parameter(
      "instance_labels_topic", "/segmentation/instance_labels");
    image_topic_ = declare_parameter("image_topic", "/camera/image_raw");
    annotated_image_topic_ = declare_parameter(
      "annotated_image_topic", "/camera/detections_annotated");
    publish_annotated_image_ = declare_parameter("publish_annotated_image", false);
    target_class_ = declare_parameter("target_class", "person");
    min_score_ = declare_parameter("min_score", 0.5);
    min_points_ = declare_parameter("min_points", 3);
    depth_window_ = declare_parameter("depth_window", 0.35);
    foreground_support_ratio_ = declare_parameter("foreground_support_ratio", 2.0);
    camera_occlusion_depth_tolerance_ = declare_parameter("camera_occlusion_depth_tolerance", 0.05);
    max_time_delta_ = declare_parameter("max_time_delta", 0.05);
    sync_queue_size_ = declare_parameter("sync_queue_size", 30);
    ground_frame_ = declare_parameter("ground_frame", "base_footprint");
    ground_config_.inlier_distance = declare_parameter(
      "ground_inlier_distance", 0.05);
    const double max_tilt_degrees = declare_parameter(
      "ground_max_tilt_degrees", 20.0);
    ground_config_.max_tilt_radians =
      max_tilt_degrees * 3.14159265358979323846 / 180.0;
    ground_config_.max_origin_distance = declare_parameter(
      "ground_max_origin_distance", 0.15);
    ground_config_.min_inliers = static_cast<std::size_t>(declare_parameter(
        "ground_min_inliers", 50));
    tracking_frame_ = declare_parameter("tracking_frame", "odom");
    if (min_points_ < 1 || depth_window_ <= 0.0 || max_time_delta_ < 0.0 ||
      !std::isfinite(foreground_support_ratio_) || foreground_support_ratio_ <= 1.0 ||
      !std::isfinite(camera_occlusion_depth_tolerance_) || camera_occlusion_depth_tolerance_ < 0.0 ||
      sync_queue_size_ < 1 || ground_config_.inlier_distance <= 0.0 ||
      max_tilt_degrees < 0.0 || ground_config_.max_origin_distance < 0.0 ||
      ground_config_.min_inliers < 3)
    {
      throw std::invalid_argument("invalid association configuration");
    }

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {camera_info_ = message;});
    labels_subscriber_.subscribe(this, labels_topic_, rmw_qos_profile_sensor_data);
    detections_subscriber_.subscribe(this, detections_topic_, rmw_qos_profile_sensor_data);
    points_subscriber_.subscribe(this, points_topic_, rmw_qos_profile_sensor_data);
    if (publish_annotated_image_) {
      image_subscriber_.subscribe(this, image_topic_, rmw_qos_profile_sensor_data);
      annotated_image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        annotated_image_topic_, rclcpp::QoS(5).reliable());
      const int annotation_queue_size = std::max(sync_queue_size_, 60);
      image_synchronizer_ =
        std::make_shared<message_filters::Synchronizer<ImageLabelDetectionCloudPolicy>>(
        ImageLabelDetectionCloudPolicy(annotation_queue_size), image_subscriber_,
        labels_subscriber_, detections_subscriber_, points_subscriber_);
      image_synchronizer_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(max_time_delta_));
      image_synchronizer_->registerCallback(std::bind(
          &LidarBboxAssociationNode::on_image_pair, this,
          std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3, std::placeholders::_4));
    } else {
      synchronizer_ =
        std::make_shared<message_filters::Synchronizer<LabelDetectionCloudPolicy>>(
        LabelDetectionCloudPolicy(sync_queue_size_), labels_subscriber_,
        detections_subscriber_, points_subscriber_);
      synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_time_delta_));
      synchronizer_->registerCallback(std::bind(
          &LidarBboxAssociationNode::on_pair, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    detections_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/fusion/detections_3d", 10);
    points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fusion/associated_points", 10);
    ground_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fusion/ground_points", 10);
    non_ground_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fusion/non_ground_points", 10);
    plane_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/fusion/ground_plane", 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/fusion/diagnostics", 10);
  }

private:
  void on_pair(
    const sensor_msgs::msg::Image::ConstSharedPtr & labels,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    process_pair(labels, detections, cloud, nullptr);
  }

  void on_image_pair(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::Image::ConstSharedPtr & labels,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    process_pair(labels, detections, cloud, image);
  }

  static std::optional<cv::Point> image_pixel(
    const Point3 & lidar, const TransformParts & camera_transform,
    const Intrinsics & intrinsics, int width, int height)
  {
    const auto camera = transform_point(
      lidar, camera_transform.rotation, camera_transform.translation);
    const auto pixel = project_point(camera, intrinsics);
    if (!pixel) {
      return std::nullopt;
    }
    const int u = static_cast<int>(std::lround((*pixel)[0]));
    const int v = static_cast<int>(std::lround((*pixel)[1]));
    if (u < 0 || v < 0 || u >= width || v >= height) {
      return std::nullopt;
    }
    return cv::Point(u, v);
  }

  static std::optional<std::uint8_t> instance_label(const std::string & id)
  {
    try {
      const auto value = std::stoul(id);
      if (value == 0 || value > 255) {
        return std::nullopt;
      }
      return static_cast<std::uint8_t>(value);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  static cv::Scalar instance_color(std::uint8_t label)
  {
    static const std::array<cv::Scalar, 6> colors{
      cv::Scalar(0, 255, 0), cv::Scalar(255, 160, 0), cv::Scalar(255, 0, 255),
      cv::Scalar(0, 220, 255), cv::Scalar(255, 80, 80), cv::Scalar(180, 255, 0)};
    return colors[(label - 1) % colors.size()];
  }

  void publish_ground_debug(
    const std_msgs::msg::Header & cloud_header,
    const std::vector<Point3> & ground_points,
    const std::vector<Point3> & non_ground_points,
    const std::optional<GroundPlane> & plane)
  {
    ground_points_publisher_->publish(point_cloud(cloud_header, ground_points));
    non_ground_points_publisher_->publish(point_cloud(cloud_header, non_ground_points));

    visualization_msgs::msg::Marker marker;
    marker.header = cloud_header;
    marker.header.frame_id = ground_frame_;
    marker.ns = "ground_plane";
    marker.id = 0;
    if (!plane) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    } else {
      const auto & c = plane->coefficients;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      // 정규화한 plane에서 원점과 가장 가까운 점 -d*n을 marker 중심으로 사용한다.
      marker.pose.position.x = -c[3] * c[0];
      marker.pose.position.y = -c[3] * c[1];
      marker.pose.position.z = -c[3] * c[2];
      const auto orientation = tf2::shortestArcQuat(
        tf2::Vector3(0.0, 0.0, 1.0), tf2::Vector3(c[0], c[1], c[2]));
      marker.pose.orientation.x = orientation.x();
      marker.pose.orientation.y = orientation.y();
      marker.pose.orientation.z = orientation.z();
      marker.pose.orientation.w = orientation.w();
      marker.scale.x = 3.0;
      marker.scale.y = 3.0;
      marker.scale.z = 0.01;
      marker.color.r = 0.1F;
      marker.color.g = 0.8F;
      marker.color.b = 0.3F;
      marker.color.a = 0.25F;
    }
    plane_publisher_->publish(marker);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header = cloud_header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_bbox_association/ground_plane";
    status.hardware_id = "simulation_lidar";
    status.level = plane ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = plane ? "valid ground plane" : "no valid plane; points preserved";
    const auto add_value = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(std::move(item));
      };
    add_value("ground_points", std::to_string(ground_points.size()));
    add_value("non_ground_points", std::to_string(non_ground_points.size()));
    add_value("inlier_distance_m", std::to_string(ground_config_.inlier_distance));
    if (plane) {
      add_value("ransac_inliers", std::to_string(plane->inlier_count));
      add_value("origin_distance_m", std::to_string(plane->origin_distance));
    }
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  void publish_association_diagnostics(const std_msgs::msg::Header & header)
  {
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_bbox_association/association";
    status.hardware_id = "simulation_camera_lidar";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "cumulative association outcomes";
    const auto add_count = [&status](const std::string & key, std::size_t value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(std::move(item));
      };
    add_count("yolo_detections", yolo_detections_);
    add_count("success", association_successes_);
    add_count("no_points", no_points_rejections_);
    add_count("multiple_depth_candidates", multiple_depth_rejections_);
    add_count("ground_rejection", ground_rejections_);
    add_count("tf_unavailable", tf_rejections_);
    // Instance 성공/거부 건수와 별개인, 처리된 cloud의 누적 가림 제외 point 수다.
    add_count("camera_occluded_points", camera_occluded_points_);
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  void process_pair(
    const sensor_msgs::msg::Image::ConstSharedPtr & labels_message,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const sensor_msgs::msg::Image::ConstSharedPtr & image)
  {
    if (!camera_info_) {
      return;
    }
    // Label 값은 Detection2D.id와 같은 inference에서 생성되므로 두 timestamp가 정확히 같아야 한다.
    if (rclcpp::Time(labels_message->header.stamp) != rclcpp::Time(detections->header.stamp)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "rejecting mismatched label/detection frames");
      return;
    }
    const auto delta_nanoseconds = std::abs(
      (rclcpp::Time(detections->header.stamp) - rclcpp::Time(cloud->header.stamp)).nanoseconds());
    if (delta_nanoseconds > rclcpp::Duration::from_seconds(max_time_delta_).nanoseconds()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting stale detection/cloud pair: %.3f s", delta_nanoseconds * 1e-9);
      return;
    }

    cv_bridge::CvImageConstPtr labels;
    try {
      labels = cv_bridge::toCvShare(labels_message, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "instance label conversion failed: %s", error.what());
      return;
    }
    if (labels->image.cols != static_cast<int>(camera_info_->width) ||
      labels->image.rows != static_cast<int>(camera_info_->height))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "instance label and CameraInfo size differ");
      return;
    }

    TransformParts camera_transform;
    TransformParts ground_transform;
    TransformParts tracking_transform;
    try {
      const rclcpp::Time stamp(cloud->header.stamp);
      // 같은 PointCloud timestamp에서 projection, ground filtering, odom 변환 TF를 조회한다.
      camera_transform = transform_parts(tf_buffer_.lookupTransform(
          camera_info_->header.frame_id, cloud->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.05)));
      ground_transform = transform_parts(tf_buffer_.lookupTransform(
          ground_frame_, cloud->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.05)));
      tracking_transform = transform_parts(tf_buffer_.lookupTransform(
          tracking_frame_, cloud->header.frame_id, stamp,
          rclcpp::Duration::from_seconds(0.05)));
    } catch (const tf2::TransformException & error) {
      ++tf_rejections_;
      publish_association_diagnostics(cloud->header);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "association TF unavailable: %s", error.what());
      return;
    }

    const Intrinsics intrinsics{
      camera_info_->k[0], camera_info_->k[4], camera_info_->k[2], camera_info_->k[5]};
    std::vector<Point3> lidar_points;
    std::vector<Point3> points_in_ground_frame;
    std::vector<std::size_t> ransac_roi;
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      const Point3 lidar{*x, *y, *z};
      lidar_points.push_back(lidar);
      points_in_ground_frame.push_back(transform_point(
          lidar, ground_transform.rotation, ground_transform.translation));
      // Camera 밖 선반이 dominant plane이 되지 않도록 Camera FoV의 point만 RANSAC에 사용한다.
      if (image_pixel(
          lidar, camera_transform, intrinsics,
          static_cast<int>(camera_info_->width), static_cast<int>(camera_info_->height)))
      {
        ransac_roi.push_back(lidar_points.size() - 1);
      }
    }

    // 사람 label과 지면 여부로 거르기 전에 모든 표면에서 가림 깊이 근거를 얻는다.
    const auto occluded = camera_occluded_points(
      lidar_points, camera_transform.rotation, camera_transform.translation,
      intrinsics, labels->image.cols, labels->image.rows, camera_occlusion_depth_tolerance_);
    camera_occluded_points_ += static_cast<std::size_t>(std::count(
        occluded.begin(), occluded.end(), true));
    const auto plane = estimate_ground_plane(
      points_in_ground_frame, ransac_roi, ground_config_);
    std::vector<Point3> ground_points;
    std::vector<Point3> non_ground_points;
    for (std::size_t index = 0; index < lidar_points.size(); ++index) {
      if (plane && point_plane_distance(points_in_ground_frame[index], *plane) <=
        ground_config_.inlier_distance)
      {
        ground_points.push_back(lidar_points[index]);
      } else {
        // 유효 plane이 없으면 아무 point도 제거하지 않는 conservative fallback이다.
        non_ground_points.push_back(lidar_points[index]);
      }
    }
    publish_ground_debug(cloud->header, ground_points, non_ground_points, plane);

    cv_bridge::CvImagePtr annotated;
    if (image) {
      try {
        annotated = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      } catch (const cv_bridge::Exception & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "annotated image conversion failed: %s", error.what());
      }
    }
    if (annotated) {
      for (const auto & point : non_ground_points) {
        const auto pixel = image_pixel(
          point, camera_transform, intrinsics, annotated->image.cols, annotated->image.rows);
        if (pixel) {
          cv::circle(annotated->image, *pixel, 1, cv::Scalar(160, 160, 160), -1, cv::LINE_AA);
        }
      }
    }

    vision_msgs::msg::Detection3DArray output;
    output.header = cloud->header;
    output.header.frame_id = tracking_frame_;
    std::vector<Point3> associated_points;
    for (const auto & detection : detections->detections) {
      const auto label = instance_label(detection.id);
      if (!label || detection.results.empty()) {
        continue;
      }
      const auto best = std::max_element(
        detection.results.begin(), detection.results.end(),
        [](const auto & left, const auto & right) {
          return left.hypothesis.score < right.hypothesis.score;
        });
      if (best->hypothesis.score < min_score_ ||
        (!target_class_.empty() && best->hypothesis.class_id != target_class_))
      {
        continue;
      }
      ++yolo_detections_;

      const auto & center = detection.bbox.center.position;
      const BoundingBox bbox{
        center.x - detection.bbox.size_x * 0.5,
        center.y - detection.bbox.size_y * 0.5,
        center.x + detection.bbox.size_x * 0.5,
        center.y + detection.bbox.size_y * 0.5};
      const auto color = instance_color(*label);
      if (annotated) {
        // YOLO 결과와 LiDAR association 결과를 구분하기 위해 bbox는 항상 표시한다.
        cv::rectangle(
          annotated->image,
          cv::Point(static_cast<int>(std::lround(bbox.u_min)),
          static_cast<int>(std::lround(bbox.v_min))),
          cv::Point(static_cast<int>(std::lround(bbox.u_max)),
          static_cast<int>(std::lround(bbox.v_max))), color, 2, cv::LINE_AA);
      }

      std::vector<Point3> instance_points;
      std::size_t points_before_ground = 0;
      for (std::size_t index = 0; index < lidar_points.size(); ++index) {
        const auto & point = lidar_points[index];
        const auto pixel = image_pixel(
          point, camera_transform, intrinsics, labels->image.cols, labels->image.rows);
        if (!occluded[index] && pixel &&
          labels->image.at<std::uint8_t>(pixel->y, pixel->x) == *label)
        {
          // 가림으로 줄어든 점을 ground_rejection으로 잘못 집계하지 않는다.
          ++points_before_ground;
          if (!plane || point_plane_distance(points_in_ground_frame[index], *plane) >
            ground_config_.inlier_distance)
          {
            instance_points.push_back(point);
          }
        }
      }
      // Mask 안의 최근접 후보가 나머지 후보보다 충분히 우세할 때만 사람 관측으로 채택한다.
      const auto analysis = analyze_instance_points(
        instance_points, camera_transform.rotation, camera_transform.translation,
        static_cast<std::size_t>(min_points_), depth_window_, foreground_support_ratio_);
      const auto & association = analysis.association;
      std::ostringstream text;
      text << "instance_" << std::setw(2) << std::setfill('0') << static_cast<int>(*label)
           << ' ' << std::fixed << std::setprecision(2) << best->hypothesis.score;
      if (!association) {
        if (points_before_ground >= static_cast<std::size_t>(min_points_) &&
          instance_points.size() < static_cast<std::size_t>(min_points_))
        {
          ++ground_rejections_;
        } else if (analysis.status == AssociationStatus::MULTIPLE_DEPTH_CANDIDATES) {
          ++multiple_depth_rejections_;
        } else {
          ++no_points_rejections_;
        }
        text << " YOLO only";
        if (annotated) {
          cv::putText(
            annotated->image, text.str(),
            cv::Point(
              std::max(0, static_cast<int>(std::lround(bbox.u_min))),
              std::max(18, static_cast<int>(std::lround(bbox.v_min)) - 6)),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
        }
        continue;
      }
      ++association_successes_;
      if (annotated) {
        for (const auto & point : association->matched_points_lidar) {
          const auto pixel = image_pixel(
            point, camera_transform, intrinsics, annotated->image.cols, annotated->image.rows);
          if (pixel) {
            cv::circle(
              annotated->image, *pixel, 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
          }
        }
        text << " range=" << association->lidar_range << "m";
        cv::putText(
          annotated->image, text.str(),
          cv::Point(
            std::max(0, static_cast<int>(std::lround(bbox.u_min))),
            std::max(18, static_cast<int>(std::lround(bbox.v_min)) - 6)),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
      }

      const Point3 tracked = transform_point(
        association->position_lidar,
        tracking_transform.rotation, tracking_transform.translation);
      vision_msgs::msg::Detection3D detection_3d;
      detection_3d.header = output.header;
      detection_3d.id = detection.id;
      detection_3d.results.push_back(*best);
      detection_3d.results.front().pose.pose.position.x = tracked.x;
      detection_3d.results.front().pose.pose.position.y = tracked.y;
      detection_3d.results.front().pose.pose.position.z = tracked.z;
      detection_3d.results.front().pose.pose.orientation.w = 1.0;
      detection_3d.bbox.center = detection_3d.results.front().pose.pose;
      output.detections.push_back(detection_3d);
      associated_points.insert(
        associated_points.end(), association->matched_points_lidar.begin(),
        association->matched_points_lidar.end());
    }

    detections_publisher_->publish(output);
    points_publisher_->publish(point_cloud(cloud->header, associated_points));
    publish_association_diagnostics(cloud->header);
    if (annotated && annotated_image_publisher_) {
      annotated->header = detections->header;
      annotated_image_publisher_->publish(*annotated->toImageMsg());
    }
  }

  std::string points_topic_;
  std::string camera_info_topic_;
  std::string detections_topic_;
  std::string labels_topic_;
  std::string image_topic_;
  std::string annotated_image_topic_;
  std::string target_class_;
  std::string ground_frame_;
  std::string tracking_frame_;
  double min_score_;
  double max_time_delta_;
  double depth_window_;
  double foreground_support_ratio_;
  double camera_occlusion_depth_tolerance_;
  int min_points_;
  int sync_queue_size_;
  bool publish_annotated_image_;
  GroundPlaneConfig ground_config_;
  std::size_t yolo_detections_{0};
  std::size_t association_successes_{0};
  std::size_t no_points_rejections_{0};
  std::size_t multiple_depth_rejections_{0};
  std::size_t ground_rejections_{0};
  std::size_t tf_rejections_{0};
  std::size_t camera_occluded_points_{0};
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::Image> labels_subscriber_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<LabelDetectionCloudPolicy>> synchronizer_;
  std::shared_ptr<message_filters::Synchronizer<ImageLabelDetectionCloudPolicy>>
  image_synchronizer_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::LidarBboxAssociationNode>());
  rclcpp::shutdown();
  return 0;
}
