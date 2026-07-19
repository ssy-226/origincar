cmake_minimum_required(VERSION 3.5)
project(qr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

add_executable(qr_node src/qr_node.cpp)

ament_target_dependencies(
  qr_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "std_msgs"
  "OpenCV"
)

target_include_directories(qr_node PRIVATE ${ZBAR_INCLUDE_DIRS})

target_link_libraries(qr_node
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

install(TARGETS qr_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
