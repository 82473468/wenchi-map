/** @ brief robo mapping by ndt
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */

#include "robo_mapping.h"

using namespace Robosense;

robo_map::robo_map(ros::NodeHandle node, ros::NodeHandle private_nh)
{

     pub_trajectory = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 10);

     private_nh.getParam("read_mapping_xml_",read_mapping_xml_);
     std::cout<<"read_mapping_xml_ "<<read_mapping_xml_<<std::endl;

     read_xml();

     ///offline_single_frame_local_lidar__mapping_pose
     save_mapping_pose_txt_name_ = save_mapping_path_;
     save_mapping_pose_txt_name_.append("mapping_pose.txt");
     save_mapping_pose_txt_.open(save_mapping_pose_txt_name_) ;
     ///offline_single_frame_local_lidar__mapping_odom
     save_mapping_trajectory_path_name_ = save_mapping_path_;
     save_mapping_trajectory_path_name_.append("mapping_odom.pcd");

     ///laser data frame counter
     cloud_frame_ = 1;

     ///previous_pose；
     previous_pose_.x    = 0;  previous_pose_.y     = 0;  previous_pose_.z   = 0;
     previous_pose_.roll = 0;  previous_pose_.pitch = 0;  previous_pose_.yaw = 0;
     ///guess_pose；
     guess_pose_.x       = 0;  guess_pose_.y        = 0;  guess_pose_.z      = 0;
     guess_pose_.roll    = 0;  guess_pose_.pitch    = 0;  guess_pose_.yaw    = 0;
     ///current_pose
     current_pose_.x     = 0;  current_pose_.y      = 0;  current_pose_.z    = 0;
     current_pose_.roll  = 0;  current_pose_.pitch  = 0;  current_pose_.yaw  = 0;

     ///transform matrix from local to global by ndt matching
     tf_ndt_local_to_global_ = Eigen::Matrix4f::Identity();

     ///main function of mapping
     mapping();
}


robo_map::~robo_map()
{};

//////////////////////////////////////////////////////////////////////////////////////////

/** @brief read config xml for mapping
*/
void robo_map::read_xml()
{
    TiXmlDocument doc ;
    if(!doc.LoadFile(read_mapping_xml_))
    {
        std::cout<<"error_xml"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("MAPPING") ;

    TiXmlElement* load_local_laser_path_Elem = node->FirstChildElement("load_local_laser_path_") ;
    TiXmlElement* save_mapping_path_Elem     = node->FirstChildElement("save_mapping_path_") ;
    TiXmlElement* start_frame_Elem           = node->FirstChildElement("start_frame_") ;
    TiXmlElement* end_frame_Elem             = node->FirstChildElement("end_frame_") ;
    TiXmlElement* frame_step_Elem            = node->FirstChildElement("frame_step_") ;
    TiXmlElement* submap_num_Elem            = node->FirstChildElement("submap_num_") ;
    TiXmlElement* min_scan_range_Elem        = node->FirstChildElement("min_scan_range_") ;
    TiXmlElement* ndt_max_iter_Elem          = node->FirstChildElement("ndt_max_iter_") ;
    TiXmlElement* ndt_cell_size_res_Elem     = node->FirstChildElement("ndt_cell_size_res_") ;
    TiXmlElement* ndt_step_size_Elem         = node->FirstChildElement("ndt_step_size_") ;
    TiXmlElement* ndt_trans_eps_Elem         = node->FirstChildElement("ndt_trans_eps_") ;

    ///the path of loading offline local laser data
    load_local_laser_path_ = load_local_laser_path_Elem->GetText();
    ///the path of saving the result of mapping
    save_mapping_path_     = save_mapping_path_Elem->GetText();
    ///laser data frame counter(start,end)
    start_frame_           = atoi(start_frame_Elem->GetText());
    end_frame_             = atoi(end_frame_Elem->GetText());
    ///the step num of input laser data
    frame_step_            = atoi(frame_step_Elem->GetText());
    ///the num of history submap frame
    submap_num_            = atoi(submap_num_Elem->GetText());
    ///ori_scan_filter_parameters
    min_scan_range_        = atof(min_scan_range_Elem->GetText());
    ///ndt parameters for scan matching
    /// Maximum iterations
    ndt_max_iter_          = atoi(ndt_max_iter_Elem->GetText());
    /// Resolution
    ndt_cell_size_res_     = atof(ndt_cell_size_res_Elem->GetText());
    /// Step size
    ndt_step_size_         = atof(ndt_step_size_Elem->GetText());
    /// Transformation epsilon
    ndt_trans_eps_         = atof(ndt_trans_eps_Elem->GetText());
}

/////////////////////////////////////////////////////////////////////////////////////////////

/** @brief main process function of mapping
*/
void robo_map::mapping()
{
    ///process single frame laser data circularly
    for(cloud_frame_ = start_frame_;cloud_frame_<end_frame_; cloud_frame_ = cloud_frame_+frame_step_)
    {
        std::cout<<"--------------------------laser frame "<<cloud_frame_<<"--------------------------"<<std::endl;
        ///load offline single frame laser data
        pcl::PointCloud<pcl::PointXYZI> cloud_point;
        std::ostringstream cloud_num;
        cloud_num<<cloud_frame_;
        std::string local_path = load_local_laser_path_ ;
        std::string cloud_num_s = cloud_num.str();
        cloud_num_s.append("laser.pcd");
        local_path.append(cloud_num_s);
        pcl::io::loadPCDFile(local_path, cloud_point);

        ///process laser data
        if (cloud_point.size() > 0)
        {
            ///filter NAN points
            pcl::PointCloud<pcl::PointXYZI> scan_ori;
            pcl::PointXYZI p;
            double r;
            for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = cloud_point.begin();
                 item != cloud_point.end(); item++)
            {
                p.x = (double) item->x;
                p.y = (double) item->y;
                p.z = (double) item->z;
                p.intensity = (double) item->intensity;

                r = p.x * p.x + p.y * p.y;
                if ((r > min_scan_range_)&&p.z>-2.5)//&&(r<max_scan_range_))
                {
                    scan_ori.push_back(p);
                }
            }

            if (cloud_frame_ == start_frame_)
            {
                std::cout << "first frame laser process" << std::endl;
                process_first_frame(scan_ori);
            }
            else
            {
                std::cout << "frame process " << std::endl;
                process_frame(scan_ori);
                std::cout << "frame_process_done " << std::endl;
            }

            ///publish current trajectory
            publish_cloud(&pub_trajectory, trajectory_);
        }
        pcl::io::savePCDFile(save_mapping_trajectory_path_name_, trajectory_);
        std::cout<<"--------------------------laser frame "<<cloud_frame_<<" process done -------------------"<<std::endl;
    }
    ///save final trajectory

}

/////////////////////////////////////////////////////////////////////////////////////////////

/** @brief process first frame laser data
 *  @param[in] scan  single frame laser data
*/
void robo_map::process_first_frame(pcl::PointCloud<pcl::PointXYZI> & scan)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    ///rotate first frame laser by tf_ndt_local_to_global_
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_ndt_local_to_global_);
    ///add first frame laser to global submap
    map_+=*transformed_scan_ptr;
    map_num_.push_back(transformed_scan_ptr->points.size());
    ///add current frame trajectory
    pcl::PointXYZ p_trajectory;
    p_trajectory.x = current_pose_.x;
    p_trajectory.y = current_pose_.y;
    p_trajectory.z = current_pose_.z;
    trajectory_.push_back(p_trajectory);
    ///save current frame pose
    save_mapping_pose_txt_<<cloud_frame_<<"   "<<current_pose_.x<<"  "<<current_pose_.y<<"  "<<
                         current_pose_.z<<"  "<<current_pose_.roll<<"  "<<current_pose_.pitch<<" "<<current_pose_.yaw<<" "
                         <<"0 "<<std::endl;
    std::cout<<"frame_frist_process "<<cloud_frame_<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////

/** @brief  process laser data
*  @param[in] scan  single frame laser data
*/
void robo_map::process_frame(pcl::PointCloud<pcl::PointXYZI> & scan)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    /// Apply voxelgrid filter
    float voxel_leaf_size = 0.3 ;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    ///load current history global submap
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));

    ///set ndt parameters
    ndt_.setTransformationEpsilon(ndt_trans_eps_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setResolution(ndt_cell_size_res_);
    ndt_.setMaximumIterations(ndt_max_iter_);
    ndt_.setInputSource(filtered_scan_ptr);
    ndt_.setInputTarget(map_ptr);

    ///generate guess pose：previous pose
    guess_pose_.x     = previous_pose_.x ;
    guess_pose_.y     = previous_pose_.y ;
    guess_pose_.z     = previous_pose_.z ;
    guess_pose_.roll  = 0 ;
    guess_pose_.pitch = 0 ;
    guess_pose_.yaw   = previous_pose_.yaw ;

    ///generate init_guess matrix according to guess_pose_
    Eigen::AngleAxisf init_rotation_x( guess_pose_.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y( guess_pose_.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z( guess_pose_.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(guess_pose_.x, guess_pose_.y, guess_pose_.z);
    Eigen::Matrix4f init_guess =
            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() ;

    ///ndt matching
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_.align(*output_cloud, init_guess);

    ///get ndt transform matrix
    tf_ndt_local_to_global_ = ndt_.getFinalTransformation();

    ///get ndt matching score(smaller,better)
    float best_score= ndt_.getFitnessScore();
    std::cout << "g_frame_num = " << cloud_frame_ << "  score= " << best_score << std::endl;

    ///get current pose according to ndt transform matrix
    tf::Matrix3x3 mat_rotate;
    mat_rotate.setValue(static_cast<double>(tf_ndt_local_to_global_(0, 0)), static_cast<double>(tf_ndt_local_to_global_(0, 1)),
                       static_cast<double>(tf_ndt_local_to_global_(0, 2)), static_cast<double>(tf_ndt_local_to_global_(1, 0)),
                       static_cast<double>(tf_ndt_local_to_global_(1, 1)), static_cast<double>(tf_ndt_local_to_global_(1, 2)),
                       static_cast<double>(tf_ndt_local_to_global_(2, 0)), static_cast<double>(tf_ndt_local_to_global_(2, 1)),
                       static_cast<double>(tf_ndt_local_to_global_(2, 2)));

    current_pose_.x = tf_ndt_local_to_global_(0, 3);
    current_pose_.y = tf_ndt_local_to_global_(1, 3);
    current_pose_.z = tf_ndt_local_to_global_(2, 3);
    mat_rotate.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw, 1);
 
    /// Update position and posture. current_pos -> previous_pos
    previous_pose_.x     = current_pose_.x;
    previous_pose_.y     = current_pose_.y;
    previous_pose_.z     = current_pose_.z;
    previous_pose_.roll  = current_pose_.roll;
    previous_pose_.pitch = current_pose_.pitch;
    previous_pose_.yaw   = current_pose_.yaw;

    std::cout << "current_pose_,x = " << current_pose_.x << std::endl;
    std::cout << "current_pose_,y = " << current_pose_.y << std::endl;
    std::cout << "current_pose_,z = " << current_pose_.z << std::endl;
    std::cout << "current_pose_,roll = " << current_pose_.roll << std::endl;
    std::cout << "current_pose_,pitch = " << current_pose_.pitch << std::endl;
    std::cout << "current_pose_,yaw = " << current_pose_.yaw << std::endl;

    ///add first frame laser to global submap
    map_ += *output_cloud;
    map_num_.push_back(output_cloud->points.size());

    ///save current frame pose
    save_mapping_pose_txt_<<cloud_frame_<<"   "<<current_pose_.x<<"  "<<current_pose_.y<<"  "<<
                         current_pose_.z<<"  "<<current_pose_.roll<<"  "<<current_pose_.pitch<<" "<<
                         current_pose_.yaw<<" "<<ndt_.getFitnessScore()<<std::endl;

    ///add current frame trajectory
    pcl::PointXYZ p_trajectory;
    p_trajectory.x = current_pose_.x ;
    p_trajectory.y = current_pose_.y ;
    p_trajectory.z = current_pose_.z ;
    trajectory_.push_back(p_trajectory);

    ///update history global submap(delete the oldest frame map)
    if(map_num_.size()>submap_num_)
    {
        map_.erase(map_.begin(),map_.begin()+map_num_.at(0));
        map_num_.erase(map_num_.begin());
    }
}
 
//////////////////////////////////////////////////////////////////////////////////////

/** @brief publish cloud msg
 *  @param[in] in_publisher  publish variable
 *  @param[in] in_cloud_to_publish_ptr  point cloud data for publishing
*/
void robo_map::publish_cloud(const ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZ> & in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = header_;
    cloud_msg.header.frame_id = "rslidar";
    in_publisher->publish(cloud_msg);
}






