cmake_minimum_required(VERSION 3.0.2)
project(easy_loam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} 	${PCL_INCLUDE_DIRS})

catkin_package()




add_library(my_lib SHARED src/odometry/point_cloud_sub.cpp)
target_link_libraries(my_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odom_node src/odometry/odom_node.cpp)
target_link_libraries(odom_node  my_lib  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
