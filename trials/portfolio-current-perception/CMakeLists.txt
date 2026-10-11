# RViz의 기존 renderer API를 사용하는 증거 이미지 추출 도구만 빌드한다.
# 입력은 export_render.cpp와 설치된 ROS 2 Jazzy·Qt 라이브러리다.
# 출력 실행 파일은 지정한 별도 build 디렉터리에 생성하며 production package는 수정하지 않는다.
cmake_minimum_required(VERSION 3.16)
project(portfolio_render_export)
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(export_render export_render.cpp)
target_compile_features(export_render PRIVATE cxx_std_17)
ament_target_dependencies(export_render rviz_common)
target_link_libraries(export_render Qt5::Widgets)
