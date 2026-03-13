cmake_minimum_required(VERSION 3.8)
project(turtlebot3_task)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Gazebo models (v3_block + coloured variants)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
