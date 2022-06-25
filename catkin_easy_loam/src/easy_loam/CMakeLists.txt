cmake_minimum_required(VERSION 3.0.2)
project(easy_loam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} 	${PCL_INCLUDE_DIRS})

catkin_package()



add_library(FeatureExtract SHARED src/odom/FeatureExtract.cpp)
add_library(LOG SHARED src/log/easylogging++.cc)

add_executable(OdomNode src/odom/OdomNode.cpp)
target_link_libraries(OdomNode  LOG FeatureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES})
