cmake_minimum_required(VERSION 3.5)
project(chase_run_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(dependencies
  rclcpp
  kobuki_ros_interfaces
  geometry_msgs
  rclcpp_cascade_lifecycle
  tf2_geometry_msgs
  tf2_ros
  tf2
  tf2_msgs
  vision_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/chase_run_cpp/PoliceThiefBehavior.cpp
  src/chase_run_cpp/ThiefBehavior.cpp
  src/chase_run_cpp/ForwardBehavior.cpp
  src/chase_run_cpp/AvoidBehavior.cpp
  src/chase_run_cpp/PoliceBehavior.cpp
  src/chase_run_cpp/SearchBehavior.cpp
  src/chase_run_cpp/PursuitBehavior.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(police_thief src/police_thief_main.cpp)
ament_target_dependencies(police_thief ${dependencies})
target_link_libraries(police_thief ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  police_thief
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
