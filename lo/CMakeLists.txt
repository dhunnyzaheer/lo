cmake_minimum_required(VERSION 3.20)
project(aru_lo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include)

include_directories(${CMAKE_BINARY_DIR}/datatypes)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(${PROJECT_NAME}_srcs
        src/scanRegistrationClass.cpp
        src/odomEstimationClass.cpp
        src/laserMappingClass.cpp
        include/scanRegistration.h
        include/lidarFactor.hpp
        include/odomEstimationClass.h
        include/laserMappingClass.h)

set(${PROJECT_NAME}_ext_libs
        Eigen3::Eigen)

set(${PROJECT_NAME}_aru_libs
        aru_datatypes_pbLaser
        aru_datatypes_pbStereoImage
        aru_datatypes_pbTransform
        aru_utilities_logging
        aru_utilities_viewer
        aru_utilities_transforms
        aru_utilities_image
        aru_utilities_laser
        ORB_SLAM3)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_srcs})
target_link_libraries(${PROJECT_NAME}
        ${${PROJECT_NAME}_ext_libs}
        ${${PROJECT_NAME}_aru_libs})

target_include_directories(${PROJECT_NAME}
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        )

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

target_include_directories(${PROJECT_NAME}
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        )
