cmake_minimum_required(VERSION 3.8)
project(wall_bot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
     src/tf_odometry_relay.py
     src/track_maker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
