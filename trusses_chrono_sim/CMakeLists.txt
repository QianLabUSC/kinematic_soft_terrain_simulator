cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)
project(trusses_chrono_sim)

#Add these directories to the search path which are needed find the package files
LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
LIST(APPEND CMAKE_PREFIX_PATH "~/chrono_workspace/lib/urdf/CMake")
LIST(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake/Chrono")

#Find all the packages, for Chrono this includes the 4 submodules we build
find_package(Chrono
	     COMPONENTS Irrlicht Vehicle Parsers Ros
             CONFIG
	     REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trusses_custom_interfaces REQUIRED)
find_package(safe_scout_simulator REQUIRED)
find_package(OpenCV REQUIRED)

#Chrono says it's important to use the same CXX standard as what it was built on
set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD ${CHRONO_CXX_STANDARD})

#These lines create and include a "sim tools" library which is all the custom class files we built.
include_directories(${CHRONO_INCLUDE_DIRS})
include_directories(include
	${rclcpp_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${geometry_msgs_INCLUDE_DIRS}
	${std_srvs_INCLUDE_DIRS}
)

add_library(sim_tools SHARED
src/sim_tools/Spirit.cpp
src/sim_tools/PIDController.cpp
src/sim_tools/Rover.cpp
src/sim_tools/ROSRover.cpp
src/sim_tools/VariableSoilParams.cpp
src/sim_tools/RHexFactory.cpp
src/sim_tools/RHex.cpp
src/sim_tools/Truss.cpp
src/sim_tools/SpiritFactory.cpp
src/sim_tools/KDTree.cpp
src/sim_tools/RoverFactory.cpp
src/sim_tools/ROSRHex.cpp
src/sim_tools/ROSSpirit.cpp
src/sim_tools/HuskyFactory.cpp
src/sim_tools/Probe.cpp)

ament_target_dependencies(sim_tools
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  tf2_ros
  trusses_custom_interfaces
  safe_scout_simulator
)

target_include_directories(sim_tools
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

#sim_tools is installed in ros2_ws/install/trusses_chrono_sim/lib as a .so file. This is why it is necessary to source the workspace before running the exectuables.
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
install(TARGETS
	sim_tools
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

macro(add_example_executable_file target)
  add_executable(${target} src/examples/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs std_srvs tf2_ros)
  install(TARGETS
  ${target}
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
  target_compile_definitions(${target} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
  target_compile_options(${target} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${target} PUBLIC ${CHRONO_LINKER_FLAGS})
  target_link_libraries(${target} sim_tools ${CHRONO_LIBRARIES})
endmacro(add_example_executable_file target)

macro(add_active_dev_executable_file target)
  add_executable(${target} src/active_development/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs std_srvs tf2_ros geometry_msgs visualization_msgs foxglove_msgs trusses_custom_interfaces)
  install(TARGETS
	${target}
	DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
  target_compile_definitions(${target} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"")
  target_compile_options(${target} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${target} PUBLIC ${CHRONO_LINKER_FLAGS})
  target_link_libraries(${target} sim_tools ${CHRONO_LIBRARIES})
endmacro(add_active_dev_executable_file target)

macro(add_experiment_executable_file target)
  add_executable(${target} src/experiments/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs std_srvs tf2_ros)
  install(TARGETS
	${target}
	DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)
  target_compile_definitions(${target} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
  target_compile_options(${target} PUBLIC ${CHRONO_CXX_FLAGS})
  target_link_options(${target} PUBLIC ${CHRONO_LINKER_FLAGS})
  target_link_libraries(${target} sim_tools ${CHRONO_LIBRARIES})
endmacro(add_experiment_executable_file target)


# add_active_dev_executable_file(spirit_experiment_probe)
# add_example_executable_file(rover_example)
# add_example_executable_file(variable_stiffness_example)

# add_active_dev_executable_file(visual_test)
# add_active_dev_executable_file(visual_params)
# add_active_dev_executable_file(husky_test)
# add_active_dev_executable_file(rover_test)
# add_active_dev_executable_file(rhex_test)
# add_active_dev_executable_file(rhex_soil_stiffness)
# add_active_dev_executable_file(rhex_soil_stiffness_ames)
add_active_dev_executable_file(rhex_soil_stiffness_path_planning)
# add_active_dev_executable_file(rhex_soil_stiffness_path_planning_shipeng)
add_active_dev_executable_file(rhex_soil_stiffness_path_planning_ros2)
target_include_directories(rhex_soil_stiffness_path_planning_ros2 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rhex_soil_stiffness_path_planning_ros2 ${OpenCV_LIBRARIES})
# add_active_dev_executable_file(shear_test)
# add_active_dev_executable_file(probe_test)
# add_active_dev_executable_file(spirit_grab_values)
# add_active_dev_executable_file(spirit_experiment) # error
# add_active_dev_executable_file(hero_video)
# add_active_dev_executable_file(spirit_experiment_probe)

# add_experiment_executable_file(spirit_test)
# add_experiment_executable_file(spirit_calibration)
# add_experiment_executable_file(spirit_calibration_zero_grav)
# add_experiment_executable_file(spirit_3truss)
# add_experiment_executable_file(2spirit_5truss_planner)
# add_experiment_executable_file(spirit_force_calibration)
# add_experiment_executable_file(spirit_force_calibration_stepped_model_verification)
# add_experiment_executable_file(1spirit_2truss_model_verification)
# add_experiment_executable_file(3spirit_3truss_model_verification)
# add_experiment_executable_file(1spirit_1truss_planner)
# add_experiment_executable_file(1spirit_2truss_planner)
# add_experiment_executable_file(1spirit_3truss_planner)
# add_experiment_executable_file(3spirit_3truss_planner)
# add_experiment_executable_file(3spirit_2truss_planner)
# add_experiment_executable_file(4spirit_3truss_planner)
#This is a ROS2 CMakeLists.txt required line
ament_package()
