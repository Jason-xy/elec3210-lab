cmake_minimum_required(VERSION 3.5)
project(ekf_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cmake_modules
        tf
        tf_conversions
        pcl_ros
        )
catkin_package()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
add_library(${PROJECT_NAME} ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(ekf_node examples/ekf_node.cpp)
target_link_libraries(ekf_node ${PROJECT_NAME})