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


add_library(LOG SHARED src/log/easylogging++.cc)


add_library(FeatureExtract SHARED src/odom/FeatureExtract.cpp)
add_executable(FeatureExtractNode src/odom/FeatureExtractNode.cpp)
target_link_libraries(FeatureExtractNode  LOG FeatureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_library(OdomCompute SHARED src/odom/OdomCompute.cpp)
add_executable(OdomComputeNode src/odom/OdomComputeNode.cpp)
target_link_libraries(OdomComputeNode  LOG OdomCompute ${catkin_LIBRARIES} ${PCL_LIBRARIES})