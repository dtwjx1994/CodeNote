cmake_minimum_required(VERSION 3.10)
project(gtsam_learning)

set(CMAKE_CXX_STANDARD 11)
# 寻找第三方库，使用大小写都可以，这里列举了两种方式
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)

# 包含第三方库头文件路径，可以使用绝对路径
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})

INCLUDE_DIRECTORIES("/usr/include/eigen3")



add_executable(gtsam_learning main.cpp)
# 链接库
target_link_libraries(gtsam_learning ${Boost_LIBRARIES}  gtsam -ltbb)
#install(TARGETS gtsam_learning RUNTIME DESTINATION bin)
add_executable(gPS_localization GPS_localization.cpp)
target_link_libraries(gPS_localization ${Boost_LIBRARIES} gtsam -ltbb)
add_executable(pose2slam pose2slam.cpp)
target_link_libraries(pose2slam ${Boost_LIBRARIES} gtsam -ltbb)
add_executable(readWriteG2O readWriteG2O.cpp)
target_link_libraries(readWriteG2O ${Boost_LIBRARIES} gtsam -ltbb)
add_executable(isam isam.cpp)
target_link_libraries(isam ${Boost_LIBRARIES} gtsam -ltbb)
add_executable(isam_laser isam_laser.cpp)
target_link_libraries(isam_laser ${Boost_LIBRARIES} gtsam -ltbb)