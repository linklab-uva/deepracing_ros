cmake_minimum_required(VERSION 3.15)
project(deepracing_launch)
find_package(ament_cmake REQUIRED)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
