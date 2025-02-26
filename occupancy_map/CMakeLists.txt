cmake_minimum_required(VERSION 3.5)
project(occupancy_map)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(OpenCV_DIR /home/aaron/CLionProjects/thirdparty/opencv/build)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

message(${OpenCV_DIR})
find_package(OpenCV REQUIRED)


add_executable(occupancy_map src/occupancy_map.cpp src/object_detector.cpp)
ament_target_dependencies(occupancy_map rclcpp sensor_msgs std_msgs cv_bridge OpenCV pcl_ros pcl_conversions)

include_directories(occupancy_map ${OpenCV_INCLUDE_DIRS})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(
        DIRECTORY include/
        DESTINATION include
)


install(
        TARGETS occupancy_map
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/occupancy_map
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
