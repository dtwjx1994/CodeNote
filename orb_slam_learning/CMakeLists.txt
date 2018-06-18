cmake_minimum_required(VERSION 3.10)
project(orb_slam_learning)

set(CMAKE_CXX_STANDARD 11)
link_libraries("~/Code/ORB_SLAM2-master/lib/libORB_SLAM2.so")
find_package(OpenCV REQUIRED)
add_executable(orb_slam_learning main.cpp)
link_directories("~/Code/ORB_SLAM2-master/include")
target_link_libraries(orb_slam_learning ${OpenCV_LIBS} ~/Code/ORB_SLAM2-master/lib/libORB_SLAM2.so)