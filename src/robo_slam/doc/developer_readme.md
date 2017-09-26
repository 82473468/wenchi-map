##开发人员文档
**日期** ： 20170905
**作者** ： 张莹莹

###1 Requirements
| column | column | 
|--------|--------|
|    Ubuntu    |   14.04.5LTS(64bit)     |

|    ROS    |   indigo    |

|    cmake    |   2.8.x    |

|    git    |   git version 1.9.1  |

|    gcc    |   4.8.4    |

|    PCL    |   1.7   |

|    Opencv    |    2.4.x    |

|    Dependency    |   g2o   |

---

###2 Project

**Build and Run**

Clone to ros workspace

`git clone http://192.168.1.20:zhangyingying/robo_mapping.git`

Compile:
`catkin build`

Note:before compiling the whole project, compile the g2o first according the readme.md in g2o file.
注意：在编译整个项目前，先依照 g2o里官方的 readme.md 编译安装 g2o 库。

Open terminal:
`roscore`

##### Package ---- robo_slam

package function : generate global pose of every frame laser data and generate the global map we need.

####2.1 save offline single frame lidar data

First of all, save offline single frame lidar data,(as mapping modules process offline single frame laser data)

- run rslidar_node or rosbag play to publish laser data

- run cloud_node to analysis laser data
- run save_single_frame_lidar_node to save laser data.

change the path of saving offline laser data path in **save_single_lidar.launch**

##### 2.1.1 *.bag data process

`rosbag play *.bag`

`roslaunch robo_slam save_single_lidar_rosbag.launch`

##### 2.1.2 *.pcap data process

 change the path of loading lidar.pcap data in **save_single_lidar.launch**, then

`roslaunch robo_slam save_single_lidar.launch`


####2.2 mapping based on NDT
This module match laser scan by NDT, then generate the pose of all the laser data (the cordinate original is set at the original of the first frame laser data)

change the parameters in **./config/robo_mapping.xml**, then

`roslaunch robo_slam robo_mapping.launch`
After mapping,
- the original pose of every frame of laser data is saved in  **mapping_pose.txt**
- the whole trajectory is saved as **mapping_odom.pcd**

####2.3 loopping based on NDT and g2o
This module perform "loop-closure detection" based on NDT and "graph-optimization" based on g2o.(eliminate accumulated error)

change the parameters in **./config/robo_loopping.xml**, then

`roslaunch robo_slam robo_loopping.launch`

After loopping,
- the optimized pose of every frame laser data is saved in **save_loop_pose.txt**
- the whole trajectory is saved as ** odom_loop.pcd**
- single frame global laser data are saved in **loop_global** folder
- loop success messages are saved in **loop_success.txt**

####2.4 generating final map
This module can generate the global map as we need.(MAP_SAVE ; VOXEL_MAP ; MAP_ROTATE)

change parameters in **./config/robo_map_generating.xml**, then

`roslaunch robo_slam map_generate_node.launch`

##### Package ---- poles_map

package function : generate poles feature map based on global pose

####2.5 detect poles
This module detects poles in every local laser data then transform the result to global pose and save the position information of every pole in global map.

change parameters in **./config/poles_detect.xml**, then

`roslaunch poles_map poles_detect.launch`

After poles detecting
- the position of every pole in global cordinate is saved in **poles.txt**


####2.6 map poles
This module merges poles to generate a global feature map

change parameters in **./config/poles_map.xml**, then

`roslaunch poles_map poles_map.launch`

After poles detecting
- the global pole feature map is saved ad **poles_map.pcd**
























