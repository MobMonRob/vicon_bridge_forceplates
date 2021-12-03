cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(vicon_bridge)
set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    dynamic_reconfigure
    geometry_msgs
    roscpp
    tf
    diagnostic_updater
)

find_package(Boost REQUIRED COMPONENTS thread)

# Generate messages and services
add_message_files(FILES
    Marker.msg
    Markers.msg
    TfDistortInfo.msg
    ForcePlate.msg
    ForcePlates.msg
)

add_service_files(FILES
    viconCalibrateSegment.srv
    viconGrabPose.srv
)

generate_messages(DEPENDENCIES geometry_msgs)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(
  cfg/tf_distort.cfg
)

catkin_package(CATKIN_DEPENDS 
    dynamic_reconfigure
    geometry_msgs
    message_runtime 
    roscpp 
)

# check for platform and use the right Vicon SDK
if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/64-bit/)
    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
#elseif(CMAKE_SIZEOF_VOID_P MATCHES "4")
#    set(VICON_SDK             vicon_sdk/Vicon_SDK_1.3_Linux/32-bit/)
#    set(VICON_SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${VICON_SDK})
#    set(VICON_SDK_LINK_DIR    ${PROJECT_SOURCE_DIR}/${VICON_SDK})
else(CMAKE_SIZEOF_VOID_P MATCHES "8")
    message(FATAL_ERROR "unable to detect platform")
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
find_library(VICON_SDK_LIBRARY ViconDataStreamSDK_CPP PATHS ${VICON_SDK_LINK_DIR})

include_directories(src ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${VICON_SDK_INCLUDE_DIR})

add_library(msvc_bridge src/msvc_bridge.cpp)

add_executable(calibrate src/calibrate_segment.cpp)
target_link_libraries(calibrate
   ${catkin_LIBRARIES}
)
add_dependencies(calibrate ${PROJECT_NAME}_gencpp)

add_executable(tf_distort src/tf_distort.cpp)
target_link_libraries(tf_distort
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(tf_distort ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

#add_executable(testclient src/ViconDataStreamSDK_CPPTest.cpp)
#target_link_libraries(testclient 
#    ${VICON_SDK_LIBRARY}
#    ${Boost_LIBRARIES}
#)

add_library(srcFiles STATIC
	src/ViconProcessor.cpp
	src/vicon_bridge.cpp
	src/RvizMarkerBuilder.cpp
)

add_executable(vicon_bridgeNodeMain
	src/vicon_bridgeNodeMain.cpp
)
target_link_libraries(vicon_bridgeNodeMain
	srcFiles
    msvc_bridge
    ${VICON_SDK_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(vicon_bridgeNodeMain ${PROJECT_NAME}_gencpp)

#add_executable(ViconRvizNodeMain
#	src/ViconRvizNodeMain.cpp
#)
#target_link_libraries(ViconRvizNodeMain
#	srcFiles
#	msvc_bridge
#    ${VICON_SDK_LIBRARY}
#    ${catkin_LIBRARIES}
#)
#add_dependencies(ViconRvizNodeMain ${PROJECT_NAME}_gencpp)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Werror=return-type -fdiagnostics-show-option")
endif()

