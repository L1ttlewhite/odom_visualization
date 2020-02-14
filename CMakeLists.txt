cmake_minimum_required(VERSION 2.8.3)
project(odom_visualization)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)
find_package(Eigen3)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(odom_visualization 
                                  src/CameraPoseVisualization.cpp
                                  src/main.cpp
                                  )
target_link_libraries(odom_visualization ${catkin_LIBRARIES})
