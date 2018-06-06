cmake_minimum_required(VERSION 3.10)
project(pcl_learning)

set(CMAKE_CXX_STANDARD 11)
#find_package(PCL 1.7.2 REQUIRED COMPONENTS)


FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04
# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcl_learning main.cpp )
target_link_libraries(pcl_learning  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(filter_passthrough filter_passthrough.cpp)
target_link_libraries(filter_passthrough ${PCL_LIBRARIES} )

add_executable(filter_downsample_voxel_grid filter_downsample_voxel_grid.cpp)
target_link_libraries(filter_downsample_voxel_grid ${PCL_LIBRARIES})

add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(cloud_viewer ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
add_executable(filter_static_outliers filter_static_outliers.cpp)
target_link_libraries(filter_static_outliers ${PCL_LIBRARIES})
add_executable(pcl2octomap  pcl2octomap.cpp)
target_link_libraries(pcl2octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(octo_mapping octo_mapping.cpp)
target_link_libraries(octo_mapping ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OpenCV_LIBS})

add_executable(registration_ICP registration_ICP.cpp)
target_link_libraries(registration_ICP ${PCL_LIBRARIES})

add_executable(boost_format boost_format.cpp)