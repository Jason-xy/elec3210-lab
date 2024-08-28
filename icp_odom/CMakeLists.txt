cmake_minimum_required(VERSION 3.5)
project(project_icp_odom)

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
include(cmake/global_definition.cmake)

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

add_executable(odom_node2 examples/odom_node.cpp)
target_link_libraries(odom_node2 ${PROJECT_NAME})

add_executable(toy_example2 examples/toy_example.cpp)
target_link_libraries(toy_example2 ${PROJECT_NAME})