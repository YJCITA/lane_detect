cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0015 OLD) # cmake 3.1以后版本新特性

 ADD_EXECUTABLE(lane_detect "./main.cpp" "lane_detect.cpp")
TARGET_LINK_LIBRARIES(lane_detect -Wl,--start-group 
						opencv_core
						opencv_imgproc
						opencv_highgui
						opencv_video
						)

