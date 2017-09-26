
cmake_minimum_required(VERSION 2.8.3)
project(robo_slam)

##########
find_package(PCL 1.7 EXACT)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        genmsg
        rospy
        roscpp
        pcl_ros
        std_msgs
        pcl_conversions

         )

find_package(OpenCV REQUIRED)

generate_messages(
        DEPENDENCIES
        std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(

        CATKIN_DEPENDS message_runtime std_msgs velodyne_pointcloud
        DEPENDS OpenCV
)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse
        g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension
        g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_viewer
        g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

#g2o_ext_csparse

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
# CSparse
FIND_PACKAGE( CSparse )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

IF(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -DUSE_FAST_PCL ${CMAKE_CXX_FLAGS}")
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")


add_executable(xml_generate_node  test/xml_generate_node.cpp )
target_link_libraries(xml_generate_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )

add_executable(save_single_frame_lidar_node src/save_single_frame_lidar.cpp )
target_link_libraries(save_single_frame_lidar_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )
 
add_executable(mapping_node test/robo_mapping_node.cpp src/robo_mapping.cpp include/robo_mapping.h)
target_link_libraries(mapping_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})

add_executable(loopping_node test/robo_loopping_node.cpp src/robo_loopping.cpp include/robo_loopping.h)
target_link_libraries(loopping_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${G2O_LIBS})

add_executable(final_map_generate_node src/map_generate_node.cpp )
target_link_libraries(final_map_generate_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )

add_executable(map_process_pub_node   src/map_process_pub.cpp )
target_link_libraries(map_process_pub_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )

add_executable(local_precise_node   src/local_precise.cpp )
target_link_libraries(local_precise_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} )


