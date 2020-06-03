cmake_minimum_required(VERSION 2.8.3)
project(single_agent_herding_adp)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  image_geometry
  sensor_msgs
  tf
  geometry_msgs
  message_generation
  std_msgs
  nav_msgs
)

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#)

catkin_package(
#  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "include"
)

add_executable(herding_adp src/helper_functions.cpp src/sheep.cpp src/bear.cpp src/dynamics_estimator.cpp src/herding_adp_main.cpp)
add_executable(arbiter_adp src/arbiter.cpp)
add_executable(ar_projector_herding_adp src/ar_projector.cpp)

add_dependencies(herding_adp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arbiter_adp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ar_projector_herding_adp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(herding_adp
  ${catkin_LIBRARIES}
)

target_link_libraries(arbiter_adp
  ${catkin_LIBRARIES}
)

target_link_libraries(ar_projector_herding_adp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
