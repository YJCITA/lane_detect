cmake_minimum_required(VERSION 2.8)
project(lane_detect)

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")  

SET(PROJECT_ROOT_PATH "${CMAKE_SOURCE_DIR}")
MESSAGE(STATUS "PROJECT_ROOT_PATH: ${PROJECT_ROOT_PATH}")
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")

# SET(OpenCV_DIR "/home/yj/bak/packet/opencv/opencv3.1/install/share/OpenCV")
# FIND_PACKAGE(OpenCV REQUIRED)
SET(OpenCV_DIR "/home/yj/bak/packet/opencv/opencv-2.4.13/install/share/OpenCV")
FIND_PACKAGE(OpenCV 2.4.13 REQUIRED)
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include(/home/yj/bak/packet/eigen3_new/script_eigen.cmake) # eigen new
MESSAGE(STATUS "EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")


ADD_DEFINITIONS(-DUSE_GLOG)

LINK_DIRECTORIES(
			"${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/include/"
            "${PROJECT_ROOT_PATH}/thirdparty/glog/linux/include/"
            "${PROJECT_ROOT_PATH}/src/ipm_yj/"
            "${PROJECT_ROOT_PATH}/"
            )
LINK_DIRECTORIES(
		"${PROJECT_ROOT_PATH}/thirdparty/gflags/linux/lib/x64/"
		"${PROJECT_ROOT_PATH}/thirdparty/glog/linux/static/x64/"
)

ADD_SUBDIRECTORY(common/time)
ADD_SUBDIRECTORY(common/base)
ADD_SUBDIRECTORY(common/system)
ADD_SUBDIRECTORY(common/concurrency)
ADD_SUBDIRECTORY(common/coordinate_trans)

ADD_SUBDIRECTORY(src/)
ADD_SUBDIRECTORY(src/ipm/)
