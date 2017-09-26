/** @ brief robo mapping by ndt
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */
#ifndef PROJECT_ROBO_MAPPING_H
#define PROJECT_ROBO_MAPPING_H

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>

#include "tinyxml.h"

namespace Robosense
{
    class robo_map{

    public:
        robo_map(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~robo_map();

        std::string read_mapping_xml_ ;

    private:

        std_msgs::Header header_ ;
        ros::Publisher   pub_trajectory ;

        ///pose of single frame laser by 6-dof
        struct pose
        {
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
        };

        ///the path of loading offline local laser data
        std::string load_local_laser_path_ ;
        ///the path of saving the result of mapping
        std::string save_mapping_path_ ;
        ///the txt name of saving the pose of mapping
        std::string save_mapping_pose_txt_name_ ;
        std::ofstream save_mapping_pose_txt_ ;
        ///the pcd name of saving the trajectory of mapping
        std::string save_mapping_trajectory_path_name_ ;

        ///laser data frame counter
        unsigned int cloud_frame_ ;
        unsigned int start_frame_ ;
        unsigned int end_frame_ ;
        ///the step num of input laser data
        unsigned int frame_step_ ;
        ///the num of history submap frame
        unsigned int submap_num_ ;

        ///ori_scan_filter_parameters
        float min_scan_range_ ;
        float max_scan_range_ ;
        float min_z_ ;

        ///mapping trajectory
        pcl::PointCloud<pcl::PointXYZ> trajectory_ ;
        ///history submap
        pcl::PointCloud<pcl::PointXYZI> map_ ;
        ///the corresponding point num of single map frame
        std::vector<int> map_num_;

        pose previous_pose_, guess_pose_ , current_pose_;

        ///ndt object
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

        ///ndt parameters for scan matching
        int   ndt_max_iter_  ;          /// Maximum iterations
        float ndt_cell_size_res_ ;      /// Resolution
        float ndt_step_size_ ;          /// Step size
        float ndt_trans_eps_ ;          /// Transformation epsilon

        ///transform matrix from local to global by ndt matching
        Eigen::Matrix4f tf_ndt_local_to_global_ ;

        //////////////////////////////////////////////////////////////////////////////////////////

        /** @brief read config xml for mapping
        */
        void read_xml();

        /** @brief main process function of mapping
        */
        void mapping();

        /** @brief process first frame laser data
        *  @param[in] scan  single frame laser data
        */
        void process_first_frame(pcl::PointCloud<pcl::PointXYZI> & scan);

        /** @brief  process laser data
        *  @param[in] scan  single frame laser data
        */
        void process_frame(pcl::PointCloud<pcl::PointXYZI> & scan);

        /** @brief publish cloud msg
         *  @param[in] in_publisher  publish variable
         *  @param[in] in_cloud_to_publish_ptr  point cloud data for publishing
        */
        void publish_cloud(const ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZ> & in_cloud_to_publish_ptr);
    };
}

#endif //PROJECT_ROBO_MAPPING_H
