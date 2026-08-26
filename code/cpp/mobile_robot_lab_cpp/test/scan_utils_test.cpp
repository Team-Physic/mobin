#include <cassert>
#include <cmath>
#include <limits>

#include "mobile_robot_lab_cpp/scan_utils.hpp"

int main()
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 2.0 * M_PI / 360.0;
  scan.range_min = 0.1;
  scan.range_max = 3.5;
  scan.ranges.assign(360, std::numeric_limits<float>::infinity());
  scan.ranges[0] = 0.4F;
  scan.ranges[30] = 0.8F;
  scan.ranges[330] = 0.6F;
  scan.ranges[5] = std::numeric_limits<float>::quiet_NaN();

  const auto result = mobile_robot_lab_cpp::sector_distances(
    scan, 15.0 * M_PI / 180.0, 60.0 * M_PI / 180.0);
  assert(result.front && std::abs(*result.front - 0.4) < 1e-6);
  assert(result.left && std::abs(*result.left - 0.8) < 1e-6);
  assert(result.right && std::abs(*result.right - 0.6) < 1e-6);

  scan.ranges.clear();
  const auto empty = mobile_robot_lab_cpp::sector_distances(
    scan, 15.0 * M_PI / 180.0, 60.0 * M_PI / 180.0);
  assert(!empty.front && !empty.left && !empty.right);
}
