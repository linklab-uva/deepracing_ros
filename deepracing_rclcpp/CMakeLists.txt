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
find_package(geometry_msgs REQUIRED)
find_package(deepracing_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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
ament_target_dependencies(tf_updater deepracing_msgs geometry_msgs rclcpp tf2 tf2_ros )
target_link_libraries(tf_updater
 utils
 Eigen3::Eigen
)


set(nodes_to_install_
  tf_updater
)


find_package(PCL QUIET COMPONENTS common)
find_package(pcl_conversions QUIET)
find_package(pcl_msgs QUIET)
find_package(jsoncpp QUIET)
if (PCL_FOUND AND pcl_conversions_FOUND AND pcl_msgs_FOUND AND jsoncpp_FOUND)
  add_executable(boundary_publisher src/boundary_publisher.cpp)
  ament_target_dependencies(boundary_publisher ament_index_cpp deepracing_msgs rclcpp pcl_conversions pcl_msgs sensor_msgs std_msgs)
  target_include_directories(boundary_publisher
  PUBLIC
  ${PCL_COMMON_INCLUDE_DIRS}
  )
  target_link_libraries(boundary_publisher
  #  ${PCL_KDTREE_LIBRARIES}
  #  ${PCL_SEARCH_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    # ${PCL_LIBRARIES}
    utils
    jsoncpp_lib
  )
  list(APPEND nodes_to_install_ boundary_publisher)
  ament_export_dependencies(pcl_msgs)
  ament_export_dependencies(pcl_conversions)
endif()


find_package(cv_bridge QUIET)
find_package(image_transport QUIET)
if((TARGET f1_datalogger::f1_datalogger) AND image_transport_FOUND AND cv_bridge_FOUND)
  add_executable(ros_rebroadcaster src/ros_rebroadcaster.cpp)
  ament_target_dependencies(ros_rebroadcaster cv_bridge image_transport sensor_msgs std_msgs deepracing_msgs)
  target_link_libraries(ros_rebroadcaster
  utils
  f1_datalogger::f1_datalogger
  )
  list(APPEND nodes_to_install_ ros_rebroadcaster)
  ament_export_dependencies(TBB)
  ament_export_dependencies(cv_bridge)
  ament_export_dependencies(image_transport)
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
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_targets(${PROJECT_NAME}-export)
ament_package()
