// Copyright 2026 JungSeong
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"

namespace mobile_robot_lab_cpp
{

using DetectionCloudPolicy = message_filters::sync_policies::ApproximateTime<
  vision_msgs::msg::Detection2DArray, sensor_msgs::msg::PointCloud2>;

struct TransformParts
{
  Rotation3 rotation;
  Point3 translation;
};

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
    target_class_ = declare_parameter("target_class", "person");
    min_score_ = declare_parameter("min_score", 0.5);
    min_points_ = declare_parameter("min_points", 3);
    max_time_delta_ = declare_parameter("max_time_delta", 0.05);
    sync_queue_size_ = declare_parameter("sync_queue_size", 30);
    ground_frame_ = declare_parameter("ground_frame", "base_footprint");
    ground_height_ = declare_parameter("ground_height", 0.05);
    tracking_frame_ = declare_parameter("tracking_frame", "odom");
    if (min_points_ < 1 || max_time_delta_ < 0.0 || sync_queue_size_ < 1) {
      throw std::invalid_argument("invalid association configuration");
    }

    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {camera_info_ = message;});
    detections_subscriber_.subscribe(this, detections_topic_, rmw_qos_profile_sensor_data);
    points_subscriber_.subscribe(this, points_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<DetectionCloudPolicy>>(
      DetectionCloudPolicy(sync_queue_size_), detections_subscriber_, points_subscriber_);
    synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_time_delta_));
    synchronizer_->registerCallback(
      std::bind(
        &LidarBboxAssociationNode::on_pair, this,
        std::placeholders::_1, std::placeholders::_2));

    detections_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/fusion/detections_3d", 10);
    points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/fusion/associated_points", 10);
  }

private:
  void on_pair(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    if (!camera_info_) {
      return;
    }
    // ROS timestamp의 정수 nanosecond를 비교해 50 ms 경계가 부동소수점 오차로 거부되지 않게 한다.
    const auto delta_nanoseconds = std::abs(
      (rclcpp::Time(detections->header.stamp) - rclcpp::Time(cloud->header.stamp)).nanoseconds());
    const auto max_delta_nanoseconds =
      rclcpp::Duration::from_seconds(max_time_delta_).nanoseconds();
    if (delta_nanoseconds > max_delta_nanoseconds) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "rejecting stale detection/cloud pair: %.3f s", delta_nanoseconds * 1e-9);
      return;
    }

    TransformParts camera_transform;
    TransformParts ground_transform;
    TransformParts tracking_transform;
    try {
      const rclcpp::Time stamp(cloud->header.stamp);
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
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "association TF unavailable: %s", error.what());
      return;
    }

    std::vector<Point3> points;
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      const Point3 lidar{*x, *y, *z};
      // 바닥 기준 z가 설정 threshold 이하인 return만 제거한다.
      if (transform_point(
          lidar, ground_transform.rotation, ground_transform.translation).z > ground_height_)
      {
        points.push_back(lidar);
      }
    }

    const Intrinsics intrinsics{
      camera_info_->k[0], camera_info_->k[4], camera_info_->k[2], camera_info_->k[5]};
    vision_msgs::msg::Detection3DArray output;
    output.header = cloud->header;
    output.header.frame_id = tracking_frame_;
    std::vector<Point3> associated_points;
    for (const auto & detection : detections->detections) {
      if (detection.results.empty()) {
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
      const auto & center = detection.bbox.center.position;
      const BoundingBox bbox{
        center.x - detection.bbox.size_x * 0.5,
        center.y - detection.bbox.size_y * 0.5,
        center.x + detection.bbox.size_x * 0.5,
        center.y + detection.bbox.size_y * 0.5};
      const auto association = associate_bbox(
        points, camera_transform.rotation, camera_transform.translation,
        intrinsics, bbox, static_cast<std::size_t>(min_points_));
      if (!association) {
        continue;
      }
      const Point3 tracked = transform_point(
        association->position_lidar,
        tracking_transform.rotation, tracking_transform.translation);
      vision_msgs::msg::Detection3D detection_3d;
      detection_3d.header = output.header;
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
  }

  std::string points_topic_;
  std::string camera_info_topic_;
  std::string detections_topic_;
  std::string target_class_;
  std::string ground_frame_;
  std::string tracking_frame_;
  double min_score_;
  double max_time_delta_;
  double ground_height_;
  int min_points_;
  int sync_queue_size_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detections_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> points_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<DetectionCloudPolicy>> synchronizer_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
};

}  // namespace mobile_robot_lab_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mobile_robot_lab_cpp::LidarBboxAssociationNode>());
  rclcpp::shutdown();
  return 0;
}
