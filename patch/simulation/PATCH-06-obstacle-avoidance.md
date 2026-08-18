# Simulation PATCH-06: 장애물 회피 노드 개선 및 검증

## 이 PATCH에서 만드는 것

기존 `turtlebot3_drive`를 작은 반응형 회피 노드로 고친다. Nav2, map, planner는 추가하지 않는다.

현재 Jazzy 소스에는 다음 문제가 있다.

- `/scan`의 `0`, `30`, `330` index를 각도로 가정한다.
- 빈 ranges에서 `.at()` 예외가 날 수 있다.
- NaN을 정상 거리처럼 사용할 수 있다.
- scan이 끊겨도 마지막 속도가 유지된다.
- 속도와 거리 기준이 compile-time macro다.
- node는 `geometry_msgs/msg/Twist`를 발행하지만 bridge는 `TwistStamped`를 구독한다.

이 PATCH에서는 bridge를 유지하고 publisher를 `TwistStamped`로 바꾼다. upstream bridge YAML을 다시 `Twist`로 되돌리는 것보다 수정 범위가 작다.

## 시작 조건

- Simulation PATCH-05의 `static`, `crossing`, `mixed` scenario가 `/scan`에 나타난다.
- 회피 없는 baseline 직진은 장애물과 접촉한다.
- 기존 package가 build된다.

```bash
cd /home/swlinux/Desktop/workspace/mobile-robot-calibration-repo/docker
docker compose run --rm shell
```

컨테이너 안에서:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
```

## 추가/수정할 파일

```text
forks/turtlebot3_simulations/turtlebot3_gazebo/
├── CMakeLists.txt
├── include/turtlebot3_gazebo/
│   ├── scan_utils.hpp
│   └── turtlebot3_drive.hpp
├── launch/turtlebot3_avoidance.launch.py
├── params/avoidance.yaml
├── src/turtlebot3_drive.cpp
└── test/scan_utils_test.cpp
```

## 1. angle 기반 sector helper를 만든다

`include/turtlebot3_gazebo/scan_utils.hpp`:

```cpp
#ifndef TURTLEBOT3_GAZEBO__SCAN_UTILS_HPP_
#define TURTLEBOT3_GAZEBO__SCAN_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <sensor_msgs/msg/laser_scan.hpp>

namespace turtlebot3_gazebo
{

struct SectorDistances
{
  std::optional<double> front;
  std::optional<double> left;
  std::optional<double> right;
};

inline void update_min(std::optional<double> & value, double range)
{
  value = value ? std::min(*value, range) : range;
}

inline SectorDistances sector_distances(
  const sensor_msgs::msg::LaserScan & scan,
  double front_half_angle,
  double side_angle)
{
  SectorDistances result;
  if (scan.ranges.empty() || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment == 0.0)
  {
    return result;
  }

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (std::isinf(range) && range > 0.0) {
      range = scan.range_max;
    }
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }

    const double raw_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double angle = std::atan2(std::sin(raw_angle), std::cos(raw_angle));

    if (std::abs(angle) <= front_half_angle) {
      update_min(result.front, range);
    } else if (angle > front_half_angle && angle <= side_angle) {
      update_min(result.left, range);
    } else if (angle < -front_half_angle && angle >= -side_angle) {
      update_min(result.right, range);
    }
  }
  return result;
}

}  // namespace turtlebot3_gazebo

#endif  // TURTLEBOT3_GAZEBO__SCAN_UTILS_HPP_
```

`atan2(sin, cos)`로 각도를 `[-pi, pi]`에 정규화하므로 scan이 `0..2pi`든 `-pi..pi`든 같은 sector가 나온다.

## 2. class 상태와 message type을 정리한다

`include/turtlebot3_gazebo/turtlebot3_drive.hpp`에서 다음을 바꾼다.

### include

```cpp
#include <array>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "turtlebot3_gazebo/scan_utils.hpp"
```

기존 `geometry_msgs/msg/twist.hpp`와 속도/state macro는 제거한다.

### publisher와 상태

```cpp
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

turtlebot3_gazebo::SectorDistances distances_;
rclcpp::Time last_scan_time_;
bool has_scan_{false};
bool avoiding_{false};

double linear_velocity_;
double angular_velocity_;
double stop_distance_;
double clear_distance_;
double side_distance_;
double scan_timeout_;
double front_half_angle_;
double side_angle_;
```

odom 기반으로 정확히 30도 회전하는 기존 state는 제거한다. 동적 장애물에는 고정 회전각보다 최신 scan을 계속 보는 반응형 동작이 더 단순하고 안전하다. `odom` subscriber가 다른 용도로 쓰이지 않으면 함께 제거한다.

함수 선언은 다음 정도만 남긴다.

```cpp
void update_callback();
void publish_velocity(double linear, double angular);
void publish_stop();
void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
```

## 3. parameter를 선언하고 검증한다

constructor 시작 부분에서 parameter를 읽는다.

```cpp
linear_velocity_ = declare_parameter("linear_velocity", 0.15);
angular_velocity_ = declare_parameter("angular_velocity", 0.8);
stop_distance_ = declare_parameter("stop_distance", 0.45);
clear_distance_ = declare_parameter("clear_distance", 0.55);
side_distance_ = declare_parameter("side_distance", 0.40);
scan_timeout_ = declare_parameter("scan_timeout", 0.5);
front_half_angle_ = declare_parameter("front_half_angle_deg", 15.0) * DEG2RAD;
side_angle_ = declare_parameter("side_angle_deg", 60.0) * DEG2RAD;

if (linear_velocity_ < 0.0 || angular_velocity_ <= 0.0 ||
  stop_distance_ <= 0.0 || clear_distance_ <= stop_distance_ ||
  side_distance_ <= 0.0 || scan_timeout_ <= 0.0 ||
  front_half_angle_ <= 0.0 || side_angle_ <= front_half_angle_ ||
  side_angle_ > M_PI)
{
  throw std::invalid_argument("invalid obstacle avoidance parameters");
}
```

필요 include를 추가한다.

```cpp
#include <stdexcept>
```

publisher는 bridge와 같은 타입으로 만든다.

```cpp
cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);
```

timer는 20 Hz면 충분하다.

```cpp
update_timer_ = create_wall_timer(50ms, std::bind(&Turtlebot3Drive::update_callback, this));
```

## 4. scan callback에서 sector를 계산한다

기존 hard-coded index loop 전체를 다음으로 바꾼다.

```cpp
void Turtlebot3Drive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  distances_ = turtlebot3_gazebo::sector_distances(
    *msg, front_half_angle_, side_angle_);
  last_scan_time_ = now();
  has_scan_ = true;
}
```

LaserScan의 양의 `Inf`는 측정 범위 안에 return이 없다는 뜻이므로 `range_max`로 취급한다. NaN, 음의 Inf, 범위 밖 값은 버린다. 그 결과에도 유효한 값이 하나도 없는 sector는 `std::nullopt`로 남기고 안전 정지한다.

## 5. velocity 발행을 `TwistStamped`로 통일한다

```cpp
void Turtlebot3Drive::publish_velocity(double linear, double angular)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = now();
  command.header.frame_id = "base_link";
  command.twist.linear.x = linear;
  command.twist.angular.z = angular;
  cmd_vel_pub_->publish(command);
}

void Turtlebot3Drive::publish_stop()
{
  publish_velocity(0.0, 0.0);
}
```

destructor에서도 best-effort zero command를 보낸다.

```cpp
Turtlebot3Drive::~Turtlebot3Drive()
{
  publish_stop();
  RCLCPP_INFO(get_logger(), "Turtlebot3 simulation node has been terminated");
}
```

주 안전장치는 destructor가 아니라 아래 watchdog이다. 프로세스가 강제 종료되면 destructor 자체가 실행되지 않을 수 있기 때문이다.

## 6. hysteresis가 있는 반응형 update를 구현한다

기존 state machine 전체를 다음 흐름으로 교체한다.

```cpp
void Turtlebot3Drive::update_callback()
{
  const bool timed_out = !has_scan_ ||
    (now() - last_scan_time_).seconds() > scan_timeout_;
  const bool invalid = !distances_.front || !distances_.left || !distances_.right;

  if (timed_out || invalid) {
    avoiding_ = true;
    publish_stop();
    return;
  }

  if (*distances_.front <= stop_distance_) {
    avoiding_ = true;
  } else if (avoiding_ && *distances_.front >= clear_distance_) {
    avoiding_ = false;
  }

  if (!avoiding_) {
    publish_velocity(linear_velocity_, 0.0);
    return;
  }

  const bool left_blocked = *distances_.left <= side_distance_;
  const bool right_blocked = *distances_.right <= side_distance_;

  if (left_blocked && right_blocked) {
    publish_stop();
  } else if (left_blocked) {
    publish_velocity(0.0, -angular_velocity_);
  } else if (right_blocked) {
    publish_velocity(0.0, angular_velocity_);
  } else if (*distances_.left >= *distances_.right) {
    publish_velocity(0.0, angular_velocity_);
  } else {
    publish_velocity(0.0, -angular_velocity_);
  }
}
```

`stop_distance < clear_distance`가 hysteresis다. 0.45 m에서 회피 상태에 들어간 뒤 전방이 0.55 m 이상 열려야 다시 전진하므로 경계에서 전진/회전을 빠르게 반복하지 않는다.

## 7. parameter YAML을 만든다

`params/avoidance.yaml`:

```yaml
turtlebot3_drive_node:
  ros__parameters:
    use_sim_time: true
    linear_velocity: 0.15
    angular_velocity: 0.8
    stop_distance: 0.45
    clear_distance: 0.55
    side_distance: 0.40
    scan_timeout: 0.5
    front_half_angle_deg: 15.0
    side_angle_deg: 60.0
```

처음에는 이 값만 사용한다. scenario마다 parameter 파일을 복제하지 않는다. 세 scenario 중 하나가 실패할 때만 공통 안전값을 조정한다.

## 8. avoidance launch에서 node를 함께 실행한다

Simulation PATCH-05의 `turtlebot3_avoidance.launch.py`에 parameter 경로와 node를 추가한다.

```python
from launch_ros.actions import Node

avoidance_params = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'params',
    'avoidance.yaml')

drive_node = Node(
    package='turtlebot3_gazebo',
    executable='turtlebot3_drive',
    name='turtlebot3_drive_node',
    output='screen',
    parameters=[avoidance_params],
)
```

import는 파일 위쪽에 추가하고, `avoidance_params`와 `drive_node` 생성은 `launch_setup` 안에서 `package_share`를 만든 뒤 추가한다. `launch_setup`이 반환하는 action 목록 마지막에 `drive_node`를 추가한다.

```python
return [
    set_resources,
    gzserver,
    gzclient,
    spawn,
    robot_state_publisher,
    drive_node,
]
```

기존 `turtlebot3_world.launch.py`에는 drive node를 자동으로 추가하지 않는다.

## 9. 가장 작은 C++ check를 추가한다

`test/scan_utils_test.cpp`:

```cpp
#include <cassert>
#include <cmath>
#include <limits>

#include "turtlebot3_gazebo/scan_utils.hpp"

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

  const auto result = turtlebot3_gazebo::sector_distances(
    scan, 15.0 * M_PI / 180.0, 60.0 * M_PI / 180.0);
  assert(result.front && std::abs(*result.front - 0.4) < 1e-6);
  assert(result.left && std::abs(*result.left - 0.8) < 1e-6);
  assert(result.right && std::abs(*result.right - 0.6) < 1e-6);

  scan.ranges.clear();
  const auto empty = turtlebot3_gazebo::sector_distances(
    scan, 15.0 * M_PI / 180.0, 60.0 * M_PI / 180.0);
  assert(!empty.front && !empty.left && !empty.right);
}
```

`CMakeLists.txt`의 `ament_package()` 앞에 추가한다.

```cmake
if(BUILD_TESTING)
  enable_testing()
  add_executable(scan_utils_test test/scan_utils_test.cpp)
  target_include_directories(scan_utils_test PRIVATE include)
  target_link_libraries(scan_utils_test ${sensor_msgs_TARGETS})
  add_test(NAME scan_utils_test COMMAND scan_utils_test)
endif()
```

새 test framework dependency는 추가하지 않는다. 이 assert check 하나가 0..2pi angle mapping, 양의 Inf를 열린 공간으로 처리, NaN 무시, 빈 scan 처리를 검증한다.

## 10. 빌드와 test를 실행한다

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
colcon test --packages-select turtlebot3_gazebo
colcon test-result --verbose
source /ws/install/setup.bash
```

publisher와 bridge type을 확인한다.

```bash
ros2 topic info /cmd_vel --verbose
```

publisher/subscriber가 모두 `geometry_msgs/msg/TwistStamped`여야 한다.

## 11. 세 scenario를 검증한다

각 실행은 simulation을 완전히 종료한 뒤 새로 시작한다.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_avoidance.launch.py scenario:=static
ros2 launch turtlebot3_gazebo turtlebot3_avoidance.launch.py scenario:=crossing
ros2 launch turtlebot3_gazebo turtlebot3_avoidance.launch.py scenario:=mixed
```

실행 중 확인한다.

```bash
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 param get /turtlebot3_drive_node stop_distance
ros2 param get /turtlebot3_drive_node scan_timeout
```

scan timeout 안전장치는 simulation을 실행한 상태에서 bridge process를 중지하거나 `/scan` remap test로 확인한다. 0.5초 안에 `/cmd_vel.twist`가 0이 되어야 한다.

각 scenario의 평가 bag은 Simulation PATCH-05의 topic 목록으로 기록한다.

## 완료 조건

- `colcon test-result --verbose`가 실패 없이 끝난다.
- `/cmd_vel` publisher와 bridge가 모두 `TwistStamped`다.
- 세 scenario에서 로봇이 장애물과 접촉하지 않는다.
- 세 scenario의 `/scan` 최소값이 0.25 m 아래로 내려가지 않는다. 더 가까워지면 속도를 낮추거나 `stop_distance`를 키운다.
- obstacle이 사라질 때까지 hysteresis가 유지된다.
- `/scan`이 끊기면 `scan_timeout` 안에 zero command가 발행된다.
- 기존 `turtlebot3_world.launch.py`는 여전히 실행된다.

## 실패할 때 확인 순서

### robot이 움직이지 않는다

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel --once
```

type mismatch와 scan invalid/timeout 로그를 먼저 확인한다. bridge YAML을 동시에 바꾸지 않는다.

### 0도 부근 장애물을 놓친다

`angle_min`, `angle_increment`, ranges size를 확인한다. index를 다시 hard-code하지 말고 `scan_utils_test`에 해당 scan convention을 한 사례 추가한다.

### 좌우로 떨린다

`clear_distance`를 `stop_distance`보다 0.1~0.2 m 크게 유지한다. 복잡한 filter를 추가하기 전에 hysteresis 폭과 angular velocity만 조정한다.

### mixed 통로에서 멈춘 채 나오지 못한다

이 반응형 controller의 한계다. 통로 자체가 로봇 폭과 stop distance에 비해 너무 좁지 않은지 먼저 확인한다. 목표점 도달과 local minimum 복구가 요구사항이 될 때만 Nav2로 확장한다.

## 이 PATCH에서 하지 않는 것

- SLAM
- global/local costmap
- 목표점 planner
- behavior tree
- learned policy

정적·횡단·혼합 장애물에 대한 안전 반응이 요구사항이면 여기서 끝낸다.
