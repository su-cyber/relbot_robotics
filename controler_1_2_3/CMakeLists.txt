cmake_minimum_required(VERSION 3.8)
project(controler_1_2_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)  # For Image message
find_package(cv_bridge REQUIRED)    # For OpenCV bridge

include_directories(include)

# Add the executable
add_executable(controler_1_2_3_node src/controler_1_2_3.cpp)
ament_target_dependencies(controler_1_2_3_node 
  rclcpp 
  std_msgs 
  example_interfaces 
  sensor_msgs 
  cv_bridge  # Add cv_bridge here
)

install(TARGETS controler_1_2_3_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
