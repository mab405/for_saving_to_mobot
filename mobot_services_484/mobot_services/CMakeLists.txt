cmake_minimum_required(VERSION 3.0.2)
project(mobot_services)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

#find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   sensor_msgs
#   message_generation
#   ${MESSAGE_DEPENDENCIES}
#)

#add_service_files(
#   FILES
#    LidarScanSrv.srv 
#   MapperSrv.srv
#  )

# generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})
# generate_messages(
#  DEPENDENCIES
#   std_msgs
#   sensor_msgs
#  )

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


cs_add_executable(dummy_percep_svc src/dummy_percep_svc.cpp)
cs_add_executable(dummy_nav_svc src/dummy_nav_svc.cpp)
cs_add_executable(dummy_manip_svc src/dummy_manip_svc.cpp)
cs_install()
cs_export()
