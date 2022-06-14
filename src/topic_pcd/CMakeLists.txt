cmake_minimum_required(VERSION 3.5)
project(topic_pcd)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_topic src/pcd_to_topic.cpp)
target_link_libraries (pcd_to_topic ${PCL_LIBRARIES})
ament_target_dependencies(pcd_to_topic rclcpp std_msgs sensor_msgs pcl_msgs)
add_executable(topic_to_pcd src/topic_to_pcd.cpp)
target_link_libraries (topic_to_pcd ${PCL_LIBRARIES})
ament_target_dependencies(topic_to_pcd rclcpp std_msgs sensor_msgs pcl_msgs)


install(TARGETS
  pcd_to_topic
  topic_to_pcd

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
