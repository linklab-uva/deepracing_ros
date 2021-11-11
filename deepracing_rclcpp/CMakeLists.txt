cmake_minimum_required(VERSION 3.15)
project(deepracing_rclcpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("#include <filesystem>
                            int main(int argc, char** argv) { std::filesystem::path p; return 0; }" HAS_STDCXXFS)

# Default to C++14 if the filesystem library isn't available
if(NOT HAS_STDCXXFS)
  set(CMAKE_CXX_STANDARD 14)
  find_package(Boost REQUIRED COMPONENTS filesystem)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time random thread)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deepracing_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

#Provides imported targets for f1_datalogger, armadillo, Boost, Eigen3, Opencv, and (on Windows) vJoy_plusplus
find_package(f1_datalogger CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(utils SHARED src/utils/f1_msg_utils.cpp src/utils/file_utils.cpp)
ament_target_dependencies(utils PUBLIC sensor_msgs std_msgs deepracing_msgs rclcpp rosidl_default_runtime)
target_link_libraries(utils
PUBLIC
  f1_datalogger::headers
  f1_datalogger::f1_datalogger_proto
  Eigen3::Eigen
)
set_target_properties(utils PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_utils)
target_include_directories(utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(utils PRIVATE "DEEPRACING_RCLCPP_BUILDING_LIBRARY")
if(NOT HAS_STDCXXFS)
  target_compile_definitions(utils PUBLIC "DEEPRACING_RCLCPP_USE_BOOST_FILESYSTEM")
  target_link_libraries(utils PUBLIC Boost::filesystem)
  ament_export_dependencies(Boost)
endif()

add_executable(tf_updater src/tf_updater.cpp)
ament_target_dependencies(tf_updater autoware_auto_msgs deepracing_msgs geometry_msgs nav_msgs random_numbers rclcpp tf2 tf2_eigen tf2_ros tf2_sensor_msgs)
target_link_libraries(tf_updater
 utils
 Eigen3::Eigen
)

add_executable(velocity_control_node src/velocity_control_node.cpp)
ament_target_dependencies(velocity_control_node control_toolbox deepracing_msgs geometry_msgs message_filters nav_msgs rclcpp std_msgs tf2_eigen)
target_link_libraries(velocity_control_node
 Eigen3::Eigen
)

set(nodes_to_install_
  tf_updater
  velocity_control_node
)


find_package(cv_bridge QUIET)
find_package(image_transport QUIET)
if((TARGET f1_datalogger::f1_datalogger) AND image_transport_FOUND AND cv_bridge_FOUND)
  add_executable(ros_rebroadcaster src/ros_rebroadcaster.cpp)
  ament_target_dependencies(ros_rebroadcaster cv_bridge image_transport sensor_msgs std_msgs std_srvs deepracing_msgs)
  target_link_libraries(ros_rebroadcaster
  utils
  f1_datalogger::f1_datalogger
  )
  list(APPEND nodes_to_install_ ros_rebroadcaster)
  ament_export_dependencies(TBB)
  ament_export_dependencies(cv_bridge)
  ament_export_dependencies(image_transport)
  ament_export_dependencies(std_srvs)
endif()
if(TARGET f1_datalogger::f1_datalogger_controllers)
  add_executable(autoware_control_node src/autoware_control_node.cpp)
  ament_target_dependencies(autoware_control_node autoware_auto_msgs control_toolbox deepracing_msgs message_filters nav_msgs rclcpp )
  target_link_libraries(autoware_control_node
  utils
  f1_datalogger::f1_datalogger_controllers
  )
  if(WIN32)
    target_compile_definitions(autoware_control_node PUBLIC "_USE_MATH_DEFINES" "NOGDI")
  endif()
  list(APPEND nodes_to_install_ autoware_control_node)
endif()



 ament_export_dependencies(ament_cmake)
 ament_export_dependencies(rclcpp)
 ament_export_dependencies(image_tools)
 ament_export_dependencies(laser_geometry)
 ament_export_dependencies(deepracing_msgs)
 ament_export_dependencies(f1_datalogger)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS ${nodes_to_install_}
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS utils
    EXPORT ${PROJECT_NAME}-export
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_targets(${PROJECT_NAME}-export)
ament_package()
