cmake_minimum_required(VERSION 3.8)
project(cv_usb_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# opencv_usb_cam executable
add_executable(opencv_usb_cam src/opencv_usb_cam.cpp)
ament_target_dependencies(opencv_usb_cam
  rclcpp
  image_transport
  cv_bridge
  camera_info_manager
  sensor_msgs
  std_msgs
)
target_link_libraries(opencv_usb_cam
  ${OpenCV_LIBRARIES}
)

# image_undistorter executable
add_executable(image_undistorter src/image_undistorter.cpp)
ament_target_dependencies(image_undistorter
  rclcpp
  image_transport
  cv_bridge
  camera_info_manager
  sensor_msgs
  std_msgs
)
target_link_libraries(image_undistorter
  ${OpenCV_LIBRARIES}
)

# Install executables
install(TARGETS
  opencv_usb_cam
  image_undistorter
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/brightness.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install camera_info
install(DIRECTORY camera_info
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
