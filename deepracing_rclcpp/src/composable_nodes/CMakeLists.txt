find_package(udp_msgs REQUIRED)

set(deepracing_composable_node_srcs
  car_setup_node.cpp
  car_status_node.cpp
  lap_data_node.cpp
  motion_data_node.cpp
  session_data_node.cpp
  telemetry_data_node.cpp
  udp_demuxer_node.cpp
)
add_library(deepracing_composable_nodes SHARED ${deepracing_composable_node_srcs})
if(WIN32)
  target_sources(deepracing_composable_nodes PRIVATE gamepad_multiplexer_node.cpp)
  target_link_libraries(deepracing_composable_nodes 
    PUBLIC 
      setupAPI 
      xinput
  )
  target_compile_definitions(deepracing_composable_nodes PUBLIC "NOGDI" "WIN32_LEAN_AND_MEAN")
  rclcpp_components_register_node(deepracing_composable_nodes
    PLUGIN "deepracing::composable_nodes::GamepadMultiplexerNode"
    EXECUTABLE gamepad_multiplexer_node_exe
  )
  if(TARGET f1_datalogger::f1_datalogger_controllers)
    target_sources(deepracing_composable_nodes PRIVATE multiagent_control_node.cpp)
    target_link_libraries(deepracing_composable_nodes PUBLIC f1_datalogger::f1_datalogger_controllers)
    rclcpp_components_register_node(deepracing_composable_nodes
      PLUGIN "deepracing::composable_nodes::MultiagentControlNode"
      EXECUTABLE multiagent_control_node_exe
    )
  endif()
endif()

ament_target_dependencies(deepracing_composable_nodes
  PUBLIC
    rclcpp
    rclcpp_components
    ament_index_cpp
    deepracing_msgs
    std_msgs
    udp_msgs
)

target_link_libraries(deepracing_composable_nodes PUBLIC utils)

target_include_directories(deepracing_composable_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(deepracing_composable_nodes PRIVATE "DEEPRACING_RCLCPP_BUILDING_LIBRARY")
