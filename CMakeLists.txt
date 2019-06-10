cmake_minimum_required(VERSION 2.8.3)
project(EuRoC_dataset_convertor)

find_package(catkin REQUIRED  
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
)

# generate dynamic reconfigure stuff
generate_dynamic_reconfigure_options(cfg/Camera.cfg)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

# we really want to use Release here
set(CMAKE_BUILD_TYPE Release)

message(STATUS "Building with ${CMAKE_BUILD_TYPE}")

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
find_package(PCL REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/include
)

# compile the ROS dependent stuff into a library
add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)

add_executable(dataset_convertor src/dataset_convertor.cpp)
target_link_libraries(dataset_convertor ${Boost_LIBRARIES}  pthread ${catkin_LIBRARIES} )

add_executable(segway_data_convertor src/segway_data_convertor.cpp src/file-system-tools.cpp)
target_link_libraries(segway_data_convertor ${Boost_LIBRARIES}  pthread ${catkin_LIBRARIES} ${PCL_LIBRARIES})


