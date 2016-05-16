cmake_minimum_required(VERSION 2.8.3)
project(number_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(number_recognition
  src/number_recognition.cpp
)
target_link_libraries(number_recognition
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_executable(training
  src/training.cpp
)
target_link_libraries(training 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


