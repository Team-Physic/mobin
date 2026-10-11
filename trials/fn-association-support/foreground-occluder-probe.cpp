// 앞쪽 비사람 물체가 person mask에 혼입된 합성 반례를 재현한다.
// 입력은 Camera optical 좌표[m]이며 앞쪽 물체 60점, 뒤쪽 사람 20점이다.
// 의미적 정답은 검증 설명에만 존재하고 association 입력에는 포함되지 않는다.
// 잘못된 앞쪽 후보를 채택하면 종료 코드 1, 거부하거나 뒤쪽을 선택하면 0이다.
// 실행 인자·환경변수는 없으며 파일을 생성하지 않는다.

#include <iostream>
#include <vector>

#include "mobile_robot_lab_cpp/camera_lidar_fusion.hpp"

int main()
{
  using namespace mobile_robot_lab_cpp;
  const Rotation3 identity{{{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}}};
  std::vector<Point3> points;
  // 두 표면 모두 영상 중심 부근에 투영된다. 점 수 차이는 사람의 증거가 아니다.
  for (int i = 0; i < 60; ++i) {
    points.push_back({0.01 * (i % 10 - 5), 0.02 * (i / 10 - 3), 2.0});
  }
  for (int i = 0; i < 20; ++i) {
    points.push_back({0.02 * (i % 5 - 2), 0.04 * (i / 5 - 2), 5.0});
  }
  const auto result = analyze_instance_points(points, identity, {0, 0, 0});
  if (!result.association) {
    std::cout << "REJECTED: no observation (person recovery unverified)\n";
    return 0;
  }
  const auto depth = result.association->position_camera.z;
  std::cout << "selected_depth_m=" << depth << " person_depth_m=5\n";
  if (depth == 2.0) {
    std::cout << "FAIL: foreground non-person accepted\n";
    return 1;
  }
  return depth == 5.0 ? 0 : 2;
}
