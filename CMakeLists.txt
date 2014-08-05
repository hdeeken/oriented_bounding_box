cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

add_definitions("-frounding-math")
add_definitions("-std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


find_package(CGAL)
if(CGAL_FOUND)
link_directories(${CGAL_LIBRARY_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})
endif(CGAL_FOUND)

find_package(GMM)
if(GMM_FOUND)
link_directories(${GMM_LIBRARY_DIRS})
include_directories(${GMM_INCLUDE_DIRS})
endif(GMM_FOUND)

#rosbuild_add_executable(obb_main src/main.cpp)
#target_link_libraries(obb_main ${CGAL_LIBRARY} ${GMM_LIBRARY} 'gmp')

rosbuild_add_executable(oriented_bounding_box_node src/oriented_bounding_box_node.cpp)
target_link_libraries(oriented_bounding_box_node ${CGAL_LIBRARY} ${GMM_LIBRARY} 'gmp')

