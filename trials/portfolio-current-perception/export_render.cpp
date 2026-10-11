// 실제 ROS topic을 기존 RViz 설정으로 표시하고 OGRE 렌더 결과만 PNG로 저장한다.
// 데스크톱 픽셀을 읽지 않으므로 다른 창·메뉴·설정 패널은 출력에 포함되지 않는다.
// 센서와 추적 계산은 변경하지 않으며 topic별 최신 표시 시각은 서로 다를 수 있다.
// Track 글자는 점 위에 표시하고 공백 폭을 글자 높이에 맞춰 ID 가림을 방지한다.
// 출력 인자: 첫 번째 필수 PNG 경로. 이후 인자는 RViz의 -d 설정과 --ros-args 계약을 따른다.
// ROS_DOMAIN_ID=87 실행에 연결한다. 저장은 시작 후 wall 15초, odom 좌표는 m다.
#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>
#include <QApplication>
#include <QTimer>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/visualizer_app.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_rendering/objects/movable_text.hpp"
#include <OgreSceneManager.h>

// Label 내용·위치는 보존하고 renderer의 가림 순서·공백 폭만 조정한다.
unsigned format_labels(Ogre::SceneNode * node)
{
  unsigned count = 0;
  for (unsigned i = 0; i < node->numAttachedObjects(); ++i) {
    auto * text = dynamic_cast<rviz_rendering::MovableText *>(node->getAttachedObject(i));
    if (text && text->getCaption().rfind("person ", 0) == 0) {
      text->showOnTop();
      text->setSpaceWidth(text->getCharacterHeight() * 0.35F);
      ++count;
    }
  }
  for (unsigned i = 0; i < node->numChildren(); ++i) {
    count += format_labels(static_cast<Ogre::SceneNode *>(node->getChild(i)));
  }
  return count;
}

int main(int argc, char ** argv)
{
  if (argc < 2 || std::filesystem::exists(argv[1])) {
    std::cerr << "supply a new output PNG path, then RViz arguments\n";
    return 1;
  }
  const std::string output = argv[1];
  // 출력 경로만 제거하고 나머지 인자는 원래 RViz 파서에 전달한다.
  for (int i = 1; i < argc; ++i) {argv[i] = argv[i + 1];}
  --argc;
  // Qt에는 UI 인자만, ROS 초기화에는 use_sim_time을 포함한 전체 인자를 전달한다.
  auto ui_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> ui_argv;
  for (auto & argument : ui_args) {ui_argv.push_back(argument.data());}
  int ui_argc = static_cast<int>(ui_argv.size());
  ui_argv.push_back(nullptr);
  QApplication app(ui_argc, ui_argv.data());
  rviz_common::VisualizerApp visualizer(
    std::make_unique<rviz_common::ros_integration::RosClientAbstraction>());
  visualizer.setApp(&app);
  if (!visualizer.init(argc, argv)) {return 2;}
  QTimer::singleShot(15000, [&]() {
    unsigned labels = 0;
    for (auto * widget : QApplication::topLevelWidgets()) {
      if (auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget)) {
        labels += format_labels(frame->getManager()->getSceneManager()->getRootSceneNode());
      }
    }
    std::cout << "readable track labels: " << labels << std::endl;
    visualizer.getRenderWindow()->renderNow();
    visualizer.getRenderWindow()->captureScreenShot(output);
    app.exit(std::filesystem::exists(output) && std::filesystem::file_size(output) > 0 ? 0 : 3);
  });
  return app.exec();
}
