cmake_minimum_required(VERSION 2.8.3)
project(data_logger CXX C)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		rospy 
		std_msgs 
		image_transport 
		cv_bridge
		sensor_msgs
		geometry_msgs
)

catkin_package()

# Usual CMakeLists
#set(OPENCV_LIB_DIR "/usr/lib/x86_64-linux-gnu" CACHE PATH " ")
#set(OPENCV_INCLUDE_DIR "/usr/include/opencv2" CACHE PATH " ")

#set(LIBS ${LIBS} ${OPENCV_LIB_DIR}/libopencv_core.so)
#set(LIBS ${LIBS} ${OPENCV_LIB_DIR}/libopencv_highgui.so)
#set(LIBS ${LIBS} ${OPENCV_LIB_DIR}/libopencv_imgproc.so)
#set(LIBS ${LIBS} ${OPENCV_LIB_DIR}/libopencv_features2d.so)

#set(includeDirs ${includeDirs} ${OPENCV_INCLUDE_DIR})
set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/data_logger.cpp)
set(SRC ${SRC} src/message_handler.cpp)
set(SRC ${SRC} include/data_logger.h)
set(SRC ${SRC} include/message_handler.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )

add_executable (data_logger ${SRC})
target_link_libraries (data_logger ${catkin_LIBRARIES} ${LIBS})
