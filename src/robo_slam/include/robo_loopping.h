/** @ brief robo loopping by ndt and g2o
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */

#ifndef PROJECT_ROBO_LOOPPING_H
#define PROJECT_ROBO_LOOPPING_H

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include <iostream>
#include <vector>

#include <sys/types.h>
#include <sys/stat.h>
#include "tinyxml.h"

namespace Robosense
{

    class robo_loop{

    public:
        robo_loop(ros::NodeHandle node,ros::NodeHandle private_nh);
        ~robo_loop();

        std::string read_loop_xml_ ;

    private:

        std_msgs::Header header_ ;
        ros::Publisher pub_trajectory ;

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

        ///keyframes
        struct key_frame
        {
            pcl::PointCloud<pcl::PointXYZI> laser_points;        ///local laser data
            pcl::PointCloud<pcl::PointXYZI> laser_points_global; ///global laser data
            pose key_pose;                                       ///pose
            Eigen::Matrix4f to_global;                           ///local to global transform
            Eigen::Matrix4f to_global_inv;                       ///global to local transform
            int key_frame_id;                                    ///key frame id
            int key_num;                                         ///exsited keyframe total num

        };

        ///the path of loading offline local laser data
        std::string load_local_lidar_path_;
        ///the path of loading mapping_pose
        std::string load_pose_path_txt_;

        ///the path of saving the result of loopping map,trajectory,pose
        std::string save_loop_global_path_;
        std::string save_loop_global_lidar_path_;
        //  std::string save_ori_global_lidar_path_;
        std::string save_loop_trajectory_path_;
        std::string loop_txt_path_;
        std::string save_loop_pose_txt_path_;

        ///saving_loop_frame_pair and matching score
        std::ofstream  loop_txt_;
        ///saving_final_loop_optimize_pose
        std::ofstream  save_loop_pose_txt_;

        ///define of g2o
        typedef g2o::BlockSolver_6_3 SlamBlockSolver;
        typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;
        ///statement of global g2o object
        g2o::SparseOptimizer globalOptimizer_;
        int optimize_step_;

        ///STL of keyframes
        std::vector<key_frame> keyframes_;

        ///STL of mapping pose data
        std::vector<std::vector<float> > pose_data_;

        ///ori_scan_filter_parameters
        float min_scan_range_;
        float max_scan_range_;
        float min_z_range_;

        ///key frame counter
        int key_id_;
        ///laser data frame counter
        int cloud_frame_;
        int start_frame_;
        int end_frame_;
        ///the step num of input laser data
        int frame_step_;

        ///zooming ratio of mapping z
        float map_z_ratio_;

        ///save final result frame num
        int save_num_;
        ///signment of final saving result
        bool save_map_final_;

        ///small range loop begin frame num
        int nearby_loop_begin_;
        ///small range loop step
        int nearby_loop_step_;
        ///small range loop success score threshold
        float nearby_loop_score_;

        ///global loop begin frame num
        int loop_begin_;
        ///global loop step
        int loop_step_;
        ///loop history submap frame num
        int loop_submap_num_;
        int loop_success_step_;
        ///global loop searching distant range threshold
        int check_loop_min_dis_;

        ///global loop success score threshold
        float loop_success_score_;
        ///global loop iterative step score min threshold
        float loop_jump_frame_score_;
        ///global loop iterative step score max threshold
        float loop_jump_frame_score_max_;

        ///loop success time counter
        int loop_times_ ;
        ///signment of whether loop success
        bool check_loop_closure_ ;

        ///loopping trajectory
        pcl::PointCloud<pcl::PointXYZ> trajectory_;

        ///zooming ratio of loopping z
        float loop_z_ratio_ ;

        pose  current_pose_;

        ///ndt parameters for scan matching
        int   ndt_max_iter_  ;          /// Maximum iterations
        float ndt_cell_size_res_ ;      /// Resolution
        float ndt_step_size_ ;          /// Step size
        float ndt_trans_eps_ ;          /// Transformation epsilon

        ///transform matrix from local to global by ndt matching
        Eigen::Matrix4f tf_ndt_local_to_global_ ;

        /////////////////////////////////////////////////////////////////////////

        /** @brief read config xml for mapping
        */
        void read_xml();

        /**@brief create folder (if the folder does not exist)
         * @param[in] directory_path  the folder path
        */
        void if_mkdir(std::string & directory_path);

        /**@brief process current line txt data(ignore useless information)
          * @param[in] file  txt data
          * @param[in] num   line index
          * @return current  line data
         */
        std::vector<float> get_txt_data(std::fstream &file, int line);

        /**@brief read current line txt data
          * @param[in] file  txt data
          * @param[in] num   line index
         */
        std::fstream& go_to_line(std::fstream &file, unsigned int num);

        /** @brief main process function of loopping
        */
        void loopping();

        /** @brief process  first frame laser data
        *  @param[in] scan  single frame laser data
        */
        void process_first_frame(pcl::PointCloud<pcl::PointXYZI> &scan);

        /** @brief  process  laser data
        *  @param[in] scan  single frame laser data
        */
        void process_frame(pcl::PointCloud<pcl::PointXYZI> &scan, std::vector<float> &current_pose_data);

        /** @brief  add constraint edge of two laser frame
         *  @param[in] infor  confidence of current pose determined by ndt mapping score
         *  @param[in] f1  keyframe of history laser frame
         *  @param[in] f2  keyframe of current laser frame
        */
        void add_g2o_edge(int &infor, key_frame &f1, key_frame &f2);

        /** @brief  check current loop
         *  @param[in] ndt_loop_score   current ndt matching score
         *  @param[in] f1  keyframe of  history laser frame
         *  @param[in] f2  keyframe of  current laser frame
         *  @param[in] check_loop_closure  signment of whether loop exsits
         *  @param[in] ndt_loop   ndt object
        */
        void check_keyframes_loop(float &ndt_loop_score, key_frame &f1, key_frame &f2,
                                  float &loop_score,
                                  bool &check_loop_closure,
                                  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> &ndt_loop);

        /** @brief  check small range loop
         *  @param[in] frames   STL of keyframes
         *  @param[in] currFrame  keyframe of current laser frame
        */
        void check_near_loops(std::vector<key_frame> &frames, key_frame &currFrame);

        /** @brief  check global loop
         *  @param[in] frames   STL of keyframes
         *  @param[in] currFrame  keyframe of current laser frame
        */
        void check_random_loops(std::vector<key_frame> &frames, key_frame &currFrame);

        /** @brief  apply final g2o global optimize
        */
        void final_optimizing();

        /** @brief publish cloud msg
         *  @param[in] in_publisher  publish variable
         *  @param[in] in_cloud_to_publish_ptr  point cloud data for publishing
        */
        void publish_cloud(const ros::Publisher *in_publisher, pcl::PointCloud<pcl::PointXYZ> &in_cloud_to_publish_ptr);
    };
}

#endif //PROJECT_ROBO_LOOPPING_H
