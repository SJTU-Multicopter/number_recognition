cmake_minimum_required(VERSION 2.8.3)
project(number_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
)

#find_package(OpenCV REQUIRED)
find_package(OpenCV)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(process
  src/process.cpp
)
target_link_libraries(process
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${OpenCV_LIBS}
)
add_executable(training
  src/training.cpp
)
target_link_libraries(training 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${OpenCV_LIBS}
)

add_executable(recognition
  src/recognition.cpp
)
target_link_libraries(recognition 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${OpenCV_LIBS}
)

add_executable(huawei
  src/huawei.cpp
)
target_link_libraries(huawei 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #${OpenCV_LIBS}
)
