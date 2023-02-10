find_package(udp_msgs REQUIRED)

add_library(deepracing_composable_nodes SHARED
  motion_data_node.cpp
)

ament_target_dependencies(deepracing_composable_nodes
  PUBLIC
    rclcpp
    rclcpp_components
    ament_index_cpp
    deepracing_msgs
    std_msgs
    udp_msgs
)

target_include_directories(deepracing_composable_nodes
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(deepracing_composable_nodes PRIVATE "DEEPRACING_RCLCPP_BUILDING_LIBRARY")

rclcpp_components_register_node(deepracing_composable_nodes
  PLUGIN "deepracing::composable_nodes::ReceiveMotionData"
  EXECUTABLE receive_motion_data
)

install(TARGETS deepracing_composable_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
