#################
# CORE LIBRARIES #
#################

set( CORE_LIBS )

list(APPEND CORE_LIBS ${CMAKE_THREAD_LIBS_INIT})

find_package(Boost 1.58 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIR})
list(APPEND CORE_LIBS ompl)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Generates source for shared message data types using protobuf
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/protofiles)
include_directories(${ProtobufIncludePath})
list(APPEND CORE_LIBS protofiles)

# ROS
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
include_directories(include ${catkin_INCLUDE_DIRS})

list(APPEND CORE_LIBS ${catkin_LIBRARIES})