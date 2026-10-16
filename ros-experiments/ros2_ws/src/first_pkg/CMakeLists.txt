cmake_minimum_required(VERSION 3.8)
project(first_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

#-----------------------------------------
find_package(rclcpp REQUIRED)
find_package(custom_messages REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(nums_decoder src/NumsDecoder.cpp)

ament_target_dependencies(nums_decoder
  rclcpp
  custom_messages
)

add_executable(ego_state_listener src/EgoStateListener.cpp)

ament_target_dependencies(ego_state_listener
  rclcpp
  nav_msgs
)
#-----------------------------------------


target_include_directories(nums_decoder PRIVATE
  include
)

install(TARGETS nums_decoder ego_state_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
