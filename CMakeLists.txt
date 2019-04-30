cmake_minimum_required(VERSION 2.6)
project(lidar_slam)
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)


find_package(OpenCV 4 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}_lib SHARED
    src/icp.cpp
    src/kmeans.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

# TEST ICP
add_executable(icp_odometry test/icp_odometry.cpp)
target_link_libraries(icp_odometry
	${PROJECT_NAME}_lib
)

# TEST KMEANS
add_executable(kmeans test/test_kmeans.cpp)
target_link_libraries(kmeans
	${PROJECT_NAME}_lib
)