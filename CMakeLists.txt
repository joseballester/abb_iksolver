cmake_minimum_required(VERSION 2.8.3)
project(abb_iksolver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# All services should go here
add_executable(abb_iksolver
  src/main_boost.cpp
)

target_include_directories(abb_iksolver PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(abb_iksolver ${catkin_LIBRARIES})
add_dependencies(abb_iksolver abb_iksolver_generate_messages_cpp)
