find_package(udp_msgs REQUIRED)

set(deepracing_composable_node_srcs
  car_setup_node.cpp
  car_status_node.cpp
  lap_data_node.cpp
  motion_data_node.cpp
  motion_data_ex_node.cpp
  session_data_node.cpp
  telemetry_data_node.cpp
  udp_demuxer_node.cpp
  map_to_baselink_transform.cpp
  map_to_odom_transform.cpp
  measurement_publisher.cpp
)
add_library(deepracing_composable_nodes SHARED ${deepracing_composable_node_srcs})

find_package(message_filters)
find_package(robot_localization)
if(message_filters_FOUND AND robot_localization_FOUND)
  target_sources(deepracing_composable_nodes PRIVATE ekf_monitor.cpp)
  set(with_ekf_monitor_node_ TRUE PARENT_SCOPE)
  ament_target_dependencies(deepracing_composable_nodes PUBLIC message_filters robot_localization)
endif()

if(WIN32)
  target_sources(deepracing_composable_nodes PRIVATE gamepad_multiplexer_node.cpp)
  target_link_libraries(deepracing_composable_nodes 
    PUBLIC 
      setupAPI 
      xinput
  )
  target_compile_definitions(deepracing_composable_nodes PUBLIC "NOGDI" "WIN32_LEAN_AND_MEAN")
  set(with_gamepad_multiplexer_ TRUE PARENT_SCOPE)
  if((TARGET f1_datalogger::f1_datalogger_controllers)  AND control_msgs_FOUND AND ackermann_msgs_FOUND)
    target_sources(deepracing_composable_nodes PRIVATE multiagent_control_node.cpp)
    target_link_libraries(deepracing_composable_nodes PUBLIC f1_datalogger::f1_datalogger_controllers)
    ament_target_dependencies(deepracing_composable_nodes PUBLIC control_msgs ackermann_msgs)
    set(with_multiagent_control_node_ TRUE PARENT_SCOPE)
  endif()
endif()

ament_target_dependencies(deepracing_composable_nodes
  PUBLIC
    rclcpp
    rclcpp_components
    ament_index_cpp
    deepracing_msgs
    geometry_msgs
    nav_msgs
    random_numbers
    std_msgs
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    udp_msgs  
)

target_link_libraries(deepracing_composable_nodes PUBLIC utils)

if(TARGET JsonCpp::JsonCpp)
  target_link_libraries(deepracing_composable_nodes PRIVATE JsonCpp::JsonCpp)
else()
  target_link_libraries(deepracing_composable_nodes PRIVATE jsoncpp_lib)
endif()

target_include_directories(deepracing_composable_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(deepracing_composable_nodes PRIVATE "DEEPRACING_RCLCPP_BUILDING_LIBRARY")
