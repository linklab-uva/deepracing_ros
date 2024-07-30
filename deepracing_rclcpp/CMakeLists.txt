cmake_minimum_required(VERSION 3.15)
project(deepracing_rclcpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
option(WITH_STDCXXFS "Use the filesystem library from the C++ standard library" ON)

set(boost_components_ date_time random thread)
if(NOT WITH_STDCXXFS)
  list(APPEND boost_components_ filesystem)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS ${boost_components_})
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(deepracing_cpp REQUIRED)
message(STATUS "deepracing_cpp_LIBRARIES: ${deepracing_cpp_LIBRARIES}")
message(STATUS "deepracing_cpp_INCLUDE_DIRS: ${deepracing_cpp_INCLUDE_DIRS}")
find_package(deepracing_msgs REQUIRED)
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp CONFIG REQUIRED)
endif()
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(udp_msgs REQUIRED)
#optional stuff
find_package(ackermann_msgs)
find_package(control_msgs)
find_package(cv_bridge)
find_package(image_transport)
find_package(control_toolbox)

#Provides imported targets for f1_datalogger, armadillo, Boost, Eigen3, Opencv, and (on Windows) vJoy_plusplus
find_package(f1_datalogger CONFIG REQUIRED)
find_package(Eigen3 CONFIG QUIET)
if(NOT (TARGET Eigen3::Eigen))
  find_package(Eigen3 REQUIRED)
endif()

add_library(utils SHARED src/utils/f1_msg_utils_2023.cpp src/utils/file_utils.cpp)
if(MSVC)
  target_sources(utils PRIVATE src/utils/xinput_msg_utils.cpp)
  target_compile_definitions(utils PRIVATE "WIN32_LEAN_AND_MEAN" "NOGDI")
endif()
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
target_compile_definitions(utils PRIVATE "DEEPRACING_RCLCPP_UTILS_BUILDING_LIBRARY")
if(NOT WITH_STDCXXFS)
  target_compile_definitions(utils PUBLIC "DEEPRACING_RCLCPP_USE_BOOST_FILESYSTEM")
  target_link_libraries(utils PUBLIC Boost::filesystem)
  ament_export_dependencies(Boost)
endif()

add_subdirectory(src/composable_nodes)

set(nodes_to_install_)

if(control_toolbox_FOUND AND ackermann_msgs_FOUND)
  add_executable(velocity_control_node src/velocity_control_node.cpp)
  ament_target_dependencies(velocity_control_node ackermann_msgs control_msgs control_toolbox deepracing_msgs geometry_msgs message_filters nav_msgs rclcpp std_msgs tf2_eigen)
  target_link_libraries(velocity_control_node
  Eigen3::Eigen
  )
  list(APPEND nodes_to_install_ velocity_control_node)
endif()

if (ackermann_msgs_FOUND)
  add_executable(race_supervisor_node src/race_supervisor.cpp)
  ament_target_dependencies(race_supervisor_node ackermann_msgs deepracing_msgs geometry_msgs message_filters nav_msgs random_numbers rclcpp std_msgs tf2_eigen)
  target_link_libraries(race_supervisor_node
  Eigen3::Eigen
  )
  list(APPEND nodes_to_install_ race_supervisor_node)
endif()

# Will update this eventually
# if((TARGET f1_datalogger::f1_datalogger) AND image_transport_FOUND AND cv_bridge_FOUND)
#   add_executable(ros_rebroadcaster src/ros_rebroadcaster.cpp)
#   ament_target_dependencies(ros_rebroadcaster cv_bridge image_transport sensor_msgs std_msgs std_srvs deepracing_msgs)
#   target_link_libraries(ros_rebroadcaster
#   utils
#   f1_datalogger::f1_datalogger
#   )
#   list(APPEND nodes_to_install_ ros_rebroadcaster)
#   if(WIN32)
#     include(${f1_datalogger_DIR}/windows_stuff.cmake)
#     get_WIN32_WINNT(win32_ver)
#     message(STATUS "Win32 version: ${win32_ver}")
#     target_compile_definitions(ros_rebroadcaster PRIVATE "_WIN32_WINNT=${win32_ver}")
#     target_link_libraries(ros_rebroadcaster RuntimeObject)
#   endif()
#   ament_export_dependencies(TBB)
#   ament_export_dependencies(cv_bridge)
#   ament_export_dependencies(image_transport)
#   ament_export_dependencies(std_srvs)
# endif()


if(TARGET f1_datalogger::f1_datalogger_controllers AND ackermann_msgs_FOUND AND control_toolbox_FOUND)
  add_executable(control_node src/control_node.cpp)
  ament_target_dependencies(control_node ackermann_msgs control_msgs control_toolbox deepracing_msgs message_filters nav_msgs rclcpp)
  target_link_libraries(control_node
  utils
  f1_datalogger::f1_datalogger_controllers
  )
  if(WIN32)
    target_compile_definitions(control_node PUBLIC "_USE_MATH_DEFINES")
  endif()
  list(APPEND nodes_to_install_ control_node)
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

if(nodes_to_install_)
  install(TARGETS ${nodes_to_install_}
      DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS utils
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include
)


rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveCarSetupData"
  EXECUTABLE receive_setup_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveCarStatusData"
  EXECUTABLE receive_status_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveLapData"
  EXECUTABLE receive_lap_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveMotionData"
  EXECUTABLE receive_motion_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveMotionExData"
  EXECUTABLE receive_motion_ex_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveSessionData"
  EXECUTABLE receive_session_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveTelemetryData"
  EXECUTABLE receive_telemetry_data_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::UdpDemuxer"
  EXECUTABLE udp_demuxer_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::MapToOdomTransform"
  EXECUTABLE map_to_odom_transform_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::MapToBaselinkTransform"
  EXECUTABLE map_to_baselink_transform_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::MeasurementPublisher"
  EXECUTABLE measurement_publisher_exe
)
rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::TrackmapPublisher"
  EXECUTABLE trackmap_publisher
)
if(with_ekf_monitor_node_)
  rclcpp_components_register_node(deepracing_composable_nodes
    PLUGIN "deepracing::composable_nodes::EkfMonitor"
    EXECUTABLE ekf_monitor_exe
  )
endif()
if(with_gamepad_multiplexer_)
  rclcpp_components_register_node(deepracing_composable_nodes
    PLUGIN "deepracing::composable_nodes::GamepadMultiplexerNode"
    EXECUTABLE gamepad_multiplexer_node_exe
  )
endif()
if(with_multiagent_control_node_)
  rclcpp_components_register_node(deepracing_composable_nodes
    PLUGIN "deepracing::composable_nodes::MultiagentControlNode"
    EXECUTABLE multiagent_control_node_exe
  )
endif()
install(TARGETS deepracing_composable_nodes
  EXPORT export_deepracing_composable_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_deepracing_composable_nodes)
ament_package()
