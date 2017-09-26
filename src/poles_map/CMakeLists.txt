cmake_minimum_required(VERSION 2.8.3)
project(poles_map)
 
catkin_package()

find_package(catkin REQUIRED COMPONENTS

        cv_bridge
        genmsg
        rospy
        roscpp
        pcl_ros
        std_msgs
        pcl_conversions


 #       message_generation
 #       std_msgs
 #       sensor_msgs
#        roscpp
  #      pcl_conversions
#        pcl_ros
#        cv_bridge
#        genmsg
#        tf
#        rospy
        )

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(

        CATKIN_DEPENDS
        DEPENDS OpenCV
)


# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

IF(PCL_VERSION VERSION_LESS "1.7.2")
    SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
    SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")



add_executable(poles_mutlistableNode src/poles_mutlistable.cpp)

target_link_libraries(poles_mutlistableNode
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(poles_map_generate_node src/global_poles_map.cpp)

target_link_libraries(poles_map_generate_node
        ${catkin_LIBRARIES}
        ${roslib_LIBRARIES}
        ${PCL_LIBRARIES}
        )


catkin_package()
 
