cmake_minimum_required(VERSION 3.8)
project(safe_scout_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trusses_custom_interfaces REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SampleGroundTruth.srv"
)

# Add executable
add_executable(ground_truth_server src/ground_truth_server.cpp)
ament_target_dependencies(ground_truth_server rclcpp std_msgs geometry_msgs foxglove_msgs ament_index_cpp)

# Link service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ground_truth_server "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  ground_truth_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/drive_sim
  scripts/rbf_visualization.py
  scripts/spatial_measurement_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
