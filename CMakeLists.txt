cmake_minimum_required(VERSION 3.0.2)
project(m2dp)

set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

add_compile_options(-std=c++11)


# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

include_directories(
    include
)


add_library(m2dp_lib m2dp.cpp)
target_link_libraries(m2dp_lib ${PCL_LIBRARIES})


add_executable(m2dp test_m2dp.cpp)
target_link_libraries(m2dp m2dp_lib ${PCL_LIBRARIES})