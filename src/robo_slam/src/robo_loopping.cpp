/** @ brief robo loopping by ndt and g2o
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */

#include "robo_loopping.h"

using namespace Robosense;

robo_loop::robo_loop(ros::NodeHandle node,ros::NodeHandle private_nh)
{

    pub_trajectory = node.advertise<sensor_msgs::PointCloud2>("trajectory", 10);

    private_nh.getParam("read_loop_xml_",read_loop_xml_);
    read_xml();

    ///the path of loading offline local laser data
    load_pose_path_txt_ = save_loop_global_path_ ;
    load_pose_path_txt_.append("mapping_pose.txt");
    ///the path of saving the result of loopping map
    save_loop_global_lidar_path_ = save_loop_global_path_ ;
    save_loop_global_lidar_path_.append("loop_global/") ;
    if_mkdir(save_loop_global_lidar_path_);
    ///the path of saving the result of trajectory
    save_loop_trajectory_path_ = save_loop_global_path_ ;
    save_loop_trajectory_path_.append("odom_loop.pcd");
    /*
    ///存储闭环优化后全局单帧点云路径
    save_ori_global_lidar_path_ = save_loop_global_path_ ;
    save_ori_global_lidar_path_.append("ori_global/") ;
    ///如果文件夹不存在,则创建文件夹
    if_mkdir(save_ori_global_lidar_path_);
    */
    ///saving_loop_frame_pair and matching score
    loop_txt_path_ = save_loop_global_path_ ;
    loop_txt_path_.append("loop_success.txt");
    loop_txt_.open(loop_txt_path_);
    ///the path of saving the result of loopping pose
    save_loop_pose_txt_path_ = save_loop_global_path_ ;
    save_loop_pose_txt_path_.append("save_loop_pose.txt") ;
    save_loop_pose_txt_.open(save_loop_pose_txt_path_) ;

    ///laser data frame counter
    cloud_frame_ = 1;
    ///key frame counter
    key_id_ = 1;
    ///zooming ratio of mapping z
    map_z_ratio_ = 0.5;

    ///small range loop begin frame num
    nearby_loop_begin_ = 20000;//2*loop_submap_num_ ;
    ///small range loop step
    nearby_loop_step_ = 20 ;
    ///small range loop success score threshold
    nearby_loop_score_ = loop_success_score_*0.6 ;

    ///global loop iterative step score min threshold
    loop_jump_frame_score_ = 6 ;
    ///global loop iterative step score max threshold
    loop_jump_frame_score_max_ = 10 ;
    ///loop success time counter
    loop_times_ = 0;
    ///signment of whether loop success
    check_loop_closure_ = false ;

    ///save final result frame num
    save_num_ =  end_frame_ - 5 ;
    ///signment of final saving result
    save_map_final_ = false ;

    ///current pose
    current_pose_.x    = 0; current_pose_.y     = 0; current_pose_.z   = 0;
    current_pose_.roll = 0; current_pose_.pitch = 0; current_pose_.yaw = 0;

    ///transform matrix from local to global by ndt matching
    tf_ndt_local_to_global_ = Eigen::Matrix4f::Identity();

    /*******************************Initialization of g2o*******************************/

    ///Initialization of solver
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver );
    globalOptimizer_.setAlgorithm( solver );
    ///no debug information out
    globalOptimizer_.setVerbose( false );
    optimize_step_ = save_num_ ;

    max_scan_range_ = 10000;
    min_z_range_ = -3;
    ///main function of loopping
    loopping();
}

robo_loop::~robo_loop()
{};

/////////////////////////////////////////////////////////////////////////

/** @brief read config xml for mapping
*/
void robo_loop::read_xml()
{
    TiXmlDocument doc;
    if(!doc.LoadFile(read_loop_xml_))
    {
        std::cout<<"error_xml"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("LOOPPING");

    TiXmlElement* load_local_lidar_path_Elem = node->FirstChildElement("load_local_lidar_path_");
    TiXmlElement* save_loop_global_path_Elem = node->FirstChildElement("save_loop_global_path_");
    TiXmlElement* start_frame_Elem           = node->FirstChildElement("start_frame_");
    TiXmlElement* end_frame_Elem             = node->FirstChildElement("end_frame_");
    TiXmlElement* frame_step_Elem            = node->FirstChildElement("frame_step_");
    TiXmlElement* min_scan_range_Elem        = node->FirstChildElement("min_scan_range_");
    TiXmlElement* ndt_max_iter_Elem          = node->FirstChildElement("ndt_max_iter_");
    TiXmlElement* ndt_cell_size_res_Elem     = node->FirstChildElement("ndt_cell_size_res_");
    TiXmlElement* ndt_step_size_Elem         = node->FirstChildElement("ndt_step_size_");
    TiXmlElement* ndt_trans_eps_Elem         = node->FirstChildElement("ndt_trans_eps_");
    TiXmlElement* loop_begin_Elem            = node->FirstChildElement("loop_begin_");
    TiXmlElement* loop_step_Elem             = node->FirstChildElement("loop_step_");
    TiXmlElement* loop_submap_num_Elem       = node->FirstChildElement("loop_submap_num_");
    TiXmlElement* loop_success_step_Elem     = node->FirstChildElement("loop_success_step_");
    TiXmlElement* check_loop_min_dis_Elem    = node->FirstChildElement("check_loop_min_dis_");
    TiXmlElement* loop_success_score_Elem    = node->FirstChildElement("loop_success_score_");
    TiXmlElement* loop_z_ratio_Elem          = node->FirstChildElement("loop_z_ratio_");

    ///loading offline local laser data path
    load_local_lidar_path_ = load_local_lidar_path_Elem->GetText();
    ///saving the result of loopping path
    save_loop_global_path_ = save_loop_global_path_Elem->GetText();
    ///laser data frame counter(start,end)
    start_frame_           = atoi(start_frame_Elem->GetText());
    end_frame_             = atoi(end_frame_Elem->GetText());
    ///the step num of input laser data
    frame_step_            = atoi(frame_step_Elem->GetText());
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
    ///global loop begin frame num
    loop_begin_            = atoi(loop_begin_Elem->GetText());
    ///global loop step
    loop_step_             = atoi(loop_step_Elem->GetText());
    ///loop history submap frame num
    loop_submap_num_       = atoi(loop_submap_num_Elem->GetText());
    ///global loop success step
    loop_success_step_     = atof(loop_success_step_Elem->GetText());
    ///global loop searching distant range threshold
    check_loop_min_dis_    = atof(check_loop_min_dis_Elem->GetText());
    ///global loop success score threshold
    loop_success_score_    = atof(loop_success_score_Elem->GetText());
    ///zooming ratio of loopping z
    loop_z_ratio_          = atof(loop_z_ratio_Elem->GetText());

}

/////////////////////////////////////////////////////////////////////////

/**@brief create folder (if the folder does not exist)
 * @param[in] directory_path  the folder path
*/
void robo_loop::if_mkdir(std::string & directory_path)
{

    char *_file = (char*)directory_path.c_str();

    if (!fopen(_file,"wb"))
    {
        std::cout<<"mkdir"<<std::endl;
        std::cout<<_file<<std::endl;
        mkdir(_file,S_IRWXU);
    }

}

/////////////////////////////////////////////////////////////////////////

/**@brief read current line txt data
  * @param[in] file  txt data
  * @param[in] num   line index
 */
std::fstream& robo_loop::go_to_line(std::fstream &file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(int p=0; p < num - 1; ++p)
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

/////////////////////////////////////////////////////////////////////////

/**@brief process current line txt data(ignore useless information)
  * @param[in] file  txt data
  * @param[in] num   line index
  * @return current  line data
 */
std::vector<float> robo_loop::get_txt_data(std::fstream &file, int line)
{

    go_to_line(file, line);
    std::string str;
    std::getline(file, str);
    std::vector<float> data;

    std::istringstream iss(str);
    int count;

    for (count=0; count<8; count++)
    {
        std::string sub;
        iss >> sub;
        double value = ::atof(sub.c_str());
        data.push_back(value);
    }

    return data;

}

/////////////////////////////////////////////////////////////////////////

/** @brief main process function of loopping
*/
void robo_loop::loopping()
{
    ///load mapping pose data
    std::fstream file(load_pose_path_txt_,std::ios_base::in);

    ///generate mapping pose data STL
    if(file.is_open())
    {
        int line_num = 1;
        float mapping_max_z = -10000;
        float mapping_min_z =  10000;
        while (line_num<end_frame_)
        {
            std::vector<float> data_line;
            data_line = get_txt_data(file, line_num);

            if(data_line[3]>mapping_max_z)
            {
                mapping_max_z = data_line[3];
            }
            else
            {
                if(data_line[3]<mapping_min_z)
                {
                    mapping_min_z = data_line[3];
                }
            }
            pose_data_.push_back(data_line);

            std::cout<<data_line[0]<<"  "<<data_line[1]<<"  "<<data_line[2]<<"  "<<data_line[3]<<"  "<<data_line[4]<<
                     "  "<<data_line[5]<<"  "<<data_line[6]<<"  "<<data_line[7]<<std::endl;

            ++line_num;
        }

        float mapping_z_dis = mapping_max_z - mapping_min_z ;

        if(mapping_z_dis>10)
        {
            for (int i = 0; i < pose_data_.size(); ++i)
            {
                pose_data_.at(i)[3] = pose_data_.at(i)[3]*map_z_ratio_;
            }
        }

    }

    ///process single frame laser data circularly: generate keyframe, check loop
    for(cloud_frame_ = start_frame_;cloud_frame_<end_frame_-1;cloud_frame_=cloud_frame_+frame_step_)
    {
        std::cout<<"--------------------------laser frame "<<cloud_frame_<<"--------------------------"<<std::endl;
        ///load offline single frame laser data
        std::ostringstream laser_num ;
        laser_num << cloud_frame_;
        std::string local_path = load_local_lidar_path_;
        std::string local_frame = laser_num.str();
        local_frame.append("laser.pcd");
        local_path.append(local_frame);
 
        pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;
        pcl::io::loadPCDFile(local_path, local_cloud_ori);

        ///process laser data
        if(local_cloud_ori.size()>0)
        {
            ///load corresponding mapping pose
            std::vector<float> current_pose_data;
            current_pose_data = pose_data_.at(cloud_frame_);
            ///filter NAN points
            pcl::PointCloud<pcl::PointXYZI> local_cloud;
            double r;
            pcl::PointXYZI p ;
            for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = local_cloud_ori.begin();
                 item != local_cloud_ori.end(); item++)
            {
                p.x = (double) item->x;
                p.y = (double) item->y;
                p.z = (double) item->z;
                p.intensity = (double) item->intensity;

                r = p.x * p.x + p.y * p.y;
                if ((r > min_scan_range_)&&(p.z>min_z_range_)&&(r<max_scan_range_))
                {
                    local_cloud.push_back(p);
                }
            }
            std::cout<<"***********the label 3"<<std::endl;
            if(cloud_frame_==start_frame_)
            {
                process_first_frame(local_cloud);
            }
            else
            {
                process_frame(local_cloud, current_pose_data);
            }

             std::cout<<"trajectory_ "<<trajectory_.size()<<std::endl;

             ///if loop successe,add loop begin frame num
             if(check_loop_closure_ == true)
             {
                 check_loop_closure_ = false;
                 loop_begin_ = cloud_frame_ + loop_success_step_;
             }

             ///apply final loop optimize
             if( cloud_frame_ > save_num_ && save_map_final_==false )
             {
                 final_optimizing();
                 save_map_final_ = true ;
             }

            ///publish current trajectory
            publish_cloud(&pub_trajectory, trajectory_);
        }
        std::cout<<"--------------------------laser frame "<<cloud_frame_<<" process done -------------------"<<std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////
/**
 * 处理第一帧数据
 * \param  scan ---输入第一帧点云数据
*/
void  robo_loop::process_first_frame(pcl::PointCloud<pcl::PointXYZI> &scan)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    ///rotate first frame laser by tf_ndt_local_to_global_
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_ndt_local_to_global_);

    ///add the first vertex to globalOptimizer
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( cloud_frame_ );
    v->setEstimate( Eigen::Isometry3d::Identity() ); ///vertex pose = indentity
    v->setFixed( true ); ///fix the first vertex
    globalOptimizer_.addVertex( v );

    ///generate keyframe
    key_frame key_f;

    key_f.key_num             = key_id_;
    key_f.key_frame_id        = cloud_frame_;
    key_f.laser_points        = *scan_ptr;
    key_f.laser_points_global = *transformed_scan_ptr;
    key_f.key_pose            = current_pose_;
    key_f.to_global           = tf_ndt_local_to_global_;
    key_f.to_global_inv       = key_f.to_global.inverse();

    keyframes_.push_back(key_f);

    ///add current trajectory
    pcl::PointXYZ p_trajectory ;
    p_trajectory.x = current_pose_.x ;
    p_trajectory.y = current_pose_.y ;
    p_trajectory.z = current_pose_.z ;
    trajectory_.push_back(p_trajectory);

    ++key_id_;
}

/////////////////////////////////////////////////////////////////////////
/**
 * 处理单帧数据
 * \param  scan ---输入单帧点云数据
*/
void robo_loop::process_frame(pcl::PointCloud<pcl::PointXYZI> &scan, std::vector<float> &current_pose_data)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    ///get current frame mapping pose and score
    float best_score= current_pose_data[7];
    current_pose_.x     = current_pose_data[1];
    current_pose_.y     = current_pose_data[2];
    current_pose_.z     = current_pose_data[3];
    current_pose_.roll  = current_pose_data[4];
    current_pose_.pitch = current_pose_data[5];
    current_pose_.yaw   = current_pose_data[6];

    ///generate transform matrix according to current pose
    Eigen::AngleAxisf current_rotation_x( current_pose_.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf current_rotation_y( current_pose_.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf current_rotation_z( current_pose_.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f current_translation(current_pose_.x, current_pose_.y, current_pose_.z);
    tf_ndt_local_to_global_ =
            (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

    std::cout << " g_frame_num = " << cloud_frame_ << "     score= " << best_score << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_ndt_local_to_global_);

    ///generate keyframe
    key_frame key_f;

    key_f.key_num             = key_id_;
    key_f.key_frame_id        = cloud_frame_;
    key_f.key_pose            = current_pose_;
    key_f.laser_points        = *scan_ptr;
    key_f.laser_points_global = *transformed_scan_ptr;
    key_f.to_global           = tf_ndt_local_to_global_;
    key_f.to_global_inv       = key_f.to_global.inverse();

    keyframes_.push_back(key_f);

    ///add current trajectory
    pcl::PointXYZ p_trajectory ;
    p_trajectory.x = current_pose_.x ;
    p_trajectory.y = current_pose_.y ;
    p_trajectory.z = current_pose_.z ;
    trajectory_.push_back(p_trajectory);

    key_id_ ++;
    std::cout << " key_frame.size = " << keyframes_.size() << std::endl;

    ///add constraint edge between current vertex and previous vertex to g2o
    ///add current vertex
    Eigen::Isometry3d vertex_transform;
    vertex_transform(0,0) =  key_f.to_global(0,0); vertex_transform(0,1) =  key_f.to_global(0,1);
    vertex_transform(0,2) =  key_f.to_global(0,2); vertex_transform(0,3) =  key_f.to_global(0,3);
    vertex_transform(1,0) =  key_f.to_global(1,0); vertex_transform(1,1) =  key_f.to_global(1,1);
    vertex_transform(1,2) =  key_f.to_global(1,2); vertex_transform(1,3) =  key_f.to_global(1,3);
    vertex_transform(2,0) =  key_f.to_global(2,0); vertex_transform(2,1) =  key_f.to_global(2,1);
    vertex_transform(2,2) =  key_f.to_global(2,2); vertex_transform(2,3) =  key_f.to_global(2,3);
    vertex_transform(3,0) =  key_f.to_global(3,0); vertex_transform(3,1) =  key_f.to_global(3,1);
    vertex_transform(3,2) =  key_f.to_global(3,2); vertex_transform(3,3) =  key_f.to_global(3,3);

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(cloud_frame_);
    v->setEstimate(vertex_transform);/// vertex pose = current pose
    globalOptimizer_.addVertex(v);

    ///confidence
    int infor = 100;
    ///determined by ndt matching score
    if(best_score>1)
    {
         infor = 20;
    }

    ///add current constraint edge
    add_g2o_edge(infor, keyframes_[keyframes_.size() - 2], key_f);

    ///check small range loop
    if(cloud_frame_>nearby_loop_begin_ && cloud_frame_%nearby_loop_step_ ==0)
    {
        check_near_loops( keyframes_, keyframes_[keyframes_.size()-1]);
    }

    ///check global loop
    if(cloud_frame_>loop_begin_ && cloud_frame_%loop_step_ ==0)
    {
        check_random_loops(keyframes_, keyframes_[keyframes_.size() - 1]);
    }
 }

/////////////////////////////////////////////////////////////////////////
/**
 * 添加当前的边约束
 * \param  infor --- 当前边的置信度(协方差矩阵的数值)
 * \param  f1    --- 当前关键帧
 * \param  f2    --- 历史关键帧
 * \param  opti  --- g2o对象
*/
void robo_loop::add_g2o_edge(int &infor, key_frame &f1, key_frame &f2)
{
    /// g2o dge
    g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
    /// set both end vertex id of the edge
    edge->setVertex(0, globalOptimizer_.vertex(f1.key_frame_id));
    edge->setVertex(1, globalOptimizer_.vertex(f2.key_frame_id));
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    ///information matrix
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    ///information matrix = covariance matrix .inverse(), representing the preliminary estimate of the accuracy of the edge
    ///as the pose is 6D,so the information matrix if 6*6,suposed that the estimate accuracy of pose is 0.1 and mutually indepent
    ///then the diagonal element = 0.01, so the information = 100
    information(0, 0) = information(1, 1) = information(2, 2) = infor;
    information(3, 3) = information(4, 4) = information(5, 5) = infor;
    ///if the thelta is set bigger,then it means the estimate accuracy of thelta is more precise
    edge->setInformation(information);
    ///the estimate of edge = relative pose between these two vertex
    Eigen::Matrix4f key_pose_transform =f1.to_global_inv*f2.to_global;
    Eigen::Isometry3d T ;
    T(0, 0) = key_pose_transform(0, 0); T(0, 1) = key_pose_transform(0, 1); T(0, 2) = key_pose_transform(0, 2); T(0, 3) = key_pose_transform(0, 3);
    T(1, 0) = key_pose_transform(1, 0); T(1, 1) = key_pose_transform(1, 1); T(1, 2) = key_pose_transform(1, 2); T(1, 3) = key_pose_transform(1, 3);
    T(2, 0) = key_pose_transform(2, 0); T(2, 1) = key_pose_transform(2, 1); T(2, 2) = key_pose_transform(2, 2); T(2, 3) = key_pose_transform(2, 3);
    T(3, 0) = key_pose_transform(3, 0); T(3, 1) = key_pose_transform(3, 1); T(3, 2) = key_pose_transform(3, 2); T(3, 3) = key_pose_transform(3, 3);

    edge->setMeasurement(T);
    ///add this edge to globalOptimizer_
    globalOptimizer_.addEdge(edge);
}

/////////////////////////////////////////////////////////////////////////
/**
 * 闭环检测匹配
 * \param  ndt_loop_score       --- 闭环匹配得分
 * \param  f1                   --- 当前关键帧
 * \param  f2                   --- 历史关键帧
 * \param  opti                 --- g2o对象
 * \param  check_loop_closure   --- 是否检测到闭环(bool)
 * \param  ndt_loop             --- ndt实例对象
*/
void robo_loop::check_keyframes_loop(float &ndt_loop_score, key_frame &f1, key_frame &f2,
                                     float &loop_score, bool &check_loop_closure,
                                     pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> &ndt_loop) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr f1_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(f1.laser_points));///history keyframe
    pcl::PointCloud<pcl::PointXYZI>::Ptr f2_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(f2.laser_points));///current keyframe

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    /// Apply voxelgrid filter
    float voxel_leaf_size = 0.5 ;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(f2_scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    ///generate history local submap
    for(int i=f1.key_num;i>f1.key_num-loop_submap_num_;i--)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(keyframes_.at(i).laser_points_global, *transformed_scan_ptr, f1.to_global_inv);
        *f1_scan_ptr+=*transformed_scan_ptr;
    }

    std::cout << "f1.size = " << f1_scan_ptr->points.size() << std::endl;
    std::cout << "f2.size = " << f2_scan_ptr->points.size() << std::endl;

    ///set ndt parameters
    ndt_loop.setTransformationEpsilon(ndt_trans_eps_);
    ndt_loop.setStepSize(ndt_step_size_);
    ndt_loop.setResolution(ndt_cell_size_res_);
    ndt_loop.setMaximumIterations(ndt_max_iter_);
    ndt_loop.setInputSource(filtered_scan_ptr);
    ndt_loop.setInputTarget(f1_scan_ptr);

    ///yaw difference between history keyframe and current key frame (adapted to different direction loop)
    float yaw_dif= f1.key_pose.yaw - f2.key_pose.yaw ;
    ///generate init_guess transform matrix according to yaw_dif
    Eigen::AngleAxisf l_rotation_x(0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf l_rotation_y(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf l_rotation_z(yaw_dif, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f l_translation(0, 0, 0);
    Eigen::Matrix4f init_guess  =
            (l_translation * l_rotation_x * l_rotation_y * l_rotation_z).matrix() ;

    ///ndt matching
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    ndt_loop.align(*output_cloud, init_guess);

    ///get ndt transform matrix
    Eigen::Matrix4f loop_transfrom(Eigen::Matrix4f::Identity());
    loop_transfrom = ndt_loop.getFinalTransformation();
    ///get ndt matching score(smaller,better)
    ndt_loop_score = ndt_loop.getFitnessScore() ;

    std::cout << "f1.key_frame_id " << f1.key_frame_id << std::endl;
    std::cout << "f2.key_frame_id " << f2.key_frame_id << std::endl;
    std::cout << "ndt_loop_score " << ndt_loop_score << std::endl;

    ///if the score is lower than the threshold, loop success, add new constrait edge and stop loop search
    if(ndt_loop_score< loop_score)
    {
        check_loop_closure = true ;
        ///save loop times and ndt matching score
        loop_times_ ++;
        loop_txt_<<loop_times_<<"  "<<f1.key_frame_id<<"  "<<f2.key_frame_id<<"   "<<ndt_loop_score<<std::endl;
        std::cout << "loop_times_ =" <<loop_times_<< std::endl;

        int reliable = 80;
        ///if the matching score is high enough, raise the confidence
        if(ndt_loop_score<loop_score*0.6)
        {
            reliable = 120;
        }
        ///edge part
        g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->setVertex(0, globalOptimizer_.vertex((f1.key_frame_id)));
        edge->setVertex(1, globalOptimizer_.vertex(f2.key_frame_id));
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        information(0, 0) = information(1, 1) = information(2, 2) = reliable;
        information(3, 3) = information(4, 4) = information(5, 5) = reliable;
        edge->setInformation(information);
        Eigen::Isometry3d T ;
        T(0, 0) = loop_transfrom(0, 0); T(0, 1) = loop_transfrom(0, 1); T(0, 2) = loop_transfrom(0, 2); T(0, 3) = loop_transfrom(0, 3);
        T(1, 0) = loop_transfrom(1, 0); T(1, 1) = loop_transfrom(1, 1); T(1, 2) = loop_transfrom(1, 2); T(1, 3) = loop_transfrom(1, 3);
        T(2, 0) = loop_transfrom(2, 0); T(2, 1) = loop_transfrom(2, 1); T(2, 2) = loop_transfrom(2, 2); T(2, 3) = loop_transfrom(2, 3);
        T(3, 0) = loop_transfrom(3, 0); T(3, 1) = loop_transfrom(3, 1); T(3, 2) = loop_transfrom(3, 2); T(3, 3) = loop_transfrom(3, 3);
        edge->setMeasurement(T);
        globalOptimizer_.addEdge(edge);
    }
}

/////////////////////////////////////////////////////////////////////////
/**
 * 近处闭环搜索
 * \param  frames               --- 关键帧序列
 * \param  currFrame            --- 当前关键帧
 * \param  opti                 --- g2o对象
*/
void robo_loop::check_near_loops(std::vector<key_frame> &frames, key_frame &currFrame)
{
    ///reverse search
    int  i    = currFrame.key_frame_id - 3 ;
    int  endi = i - 6 ;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_loop;
    float ndt_loop_score = 0 ;
    ///闭环搜索循环
    while( i>endi )
    {
        std::cout << "near_loop" << frames[i].key_frame_id << std::endl;
        bool check_near_closure = false;
        check_keyframes_loop(ndt_loop_score, frames[i], currFrame,nearby_loop_score_, check_near_closure,ndt_loop);
        --i;
        std::cout << "check_near_loop_closure = " << check_near_closure << std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////
/**
 * 闭环搜索
 * \param  frames               --- 关键帧序列
 * \param  currFrame            --- 当前关键帧
 * \param  opti                 --- g2o对象
*/
void robo_loop::check_random_loops(std::vector<key_frame> &frames, key_frame &currFrame)
{
    ///reverse search
    int  i = loop_submap_num_ + 2 ;
    int  endi = frames.size() - 100;
    int  jump_step = loop_step_ ;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_loop;
    float ndt_loop_score = 0 ;
    ///loop search circularly
    while( i<endi && check_loop_closure_ ==false)
    {
        std::cout << "loop" << frames[i].key_frame_id << std::endl;
        ///calculate the distant of pose between current keyframe and history keyframe
        float disX = frames[i].key_pose.x - currFrame.key_pose.x ;
        float disY = frames[i].key_pose.y - currFrame.key_pose.y ;
        float dis = disX*disX + disY*disY ;
        std::cout << "dis = " << dis << std::endl;
        ///if dis is lower than threshold,apply loop check
        if(dis < check_loop_min_dis_)
        {
            check_keyframes_loop(ndt_loop_score, frames[i], currFrame, loop_success_score_, check_loop_closure_, ndt_loop);

            ///change loop step iteratively according the matching score, similarly to Gradient decent method
            jump_step = 2*loop_step_;
            ///if the matching score is 匹配分数决定迭代步长.类似于梯度下降法
            if (ndt_loop_score < loop_jump_frame_score_)
            {
                jump_step = loop_step_;
            }
            else
            {
                if (ndt_loop_score > loop_jump_frame_score_max_)
                {
                    jump_step = 4*loop_step_;
                }
            }
        }
        i = i + jump_step ;
    }
    std::cout << "check_loop_closure_ = " << check_loop_closure_ << std::endl;
}

/////////////////////////////////////////////////////////////////////////
/**
 * 执行全局g2o优化,并存储优化结果
*/
void robo_loop::final_optimizing()
{
    std::cout << "optimizing pose graph, vertices: " << globalOptimizer_.vertices().size() << std::endl;
    //globalOptimizer_.save(g2o_before_txt);
    globalOptimizer_.initializeOptimization();
    globalOptimizer_.optimize(optimize_step_); //可以指定优化步数
    // globalOptimizer_.save( g2o_after_txt);
    std::cout << "Optimization done." << std::endl;

    ///clear the old trajectory_
    trajectory_.clear();

    for (size_t i = 1; i< keyframes_.size(); i++)
    {
        ///get a vertex data from g2o
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer_.vertex( keyframes_[i].key_frame_id ));
        ///get the pose after optimizing
        Eigen::Isometry3d pose_optimize = vertex->estimate();
        ///get the corresponding original local laser data from STL of the keyframes
        pcl::PointCloud<pcl::PointXYZI>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZI>(keyframes_[i].laser_points));

        std::cout<<"i "<<i<<std::endl;
        std::cout<<"vertex_pose "<< pose_optimize.matrix()<<std::endl;

        tf::Matrix3x3 mat_z;
        mat_z.setValue(static_cast<double>(pose_optimize.matrix()(0, 0)),
                       static_cast<double>(pose_optimize.matrix()(0, 1)),
                       static_cast<double>(pose_optimize.matrix()(0, 2)),
                       static_cast<double>(pose_optimize.matrix()(1, 0)),
                       static_cast<double>(pose_optimize.matrix()(1, 1)),
                       static_cast<double>(pose_optimize.matrix()(1, 2)),
                       static_cast<double>(pose_optimize.matrix()(2, 0)),
                       static_cast<double>(pose_optimize.matrix()(2, 1)),
                       static_cast<double>(pose_optimize.matrix()(2, 2)));
        pose v_p;
        ///get the optimized pose and zoom the z value
        v_p.x = pose_optimize.matrix()(0, 3);
        v_p.y = pose_optimize.matrix()(1, 3);
        v_p.z = pose_optimize.matrix()(2, 3)*loop_z_ratio_;
        mat_z.getRPY(v_p.roll, v_p.pitch, v_p.yaw, 1);

        ///generate loop trajectory
        pcl::PointXYZ p_trajectory;
        p_trajectory.x = v_p.x ;
        p_trajectory.y = v_p.y ;
        p_trajectory.z = v_p.z ;
        trajectory_.push_back(p_trajectory);

        ///save the pose after optimizing
        save_loop_pose_txt_<<i<<"    "<<v_p.x<<"  "<<v_p.y<<"  "<<v_p.z<<"  "<<v_p.roll<<"   "<<v_p.pitch<<"  "<<v_p.yaw
                          <<"    "<<pose_optimize.matrix()(2, 3)<<std::endl;

        ///generate transform matrix according to the optimized pose
        Eigen::AngleAxisf v_pose_rotation_x(v_p.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf v_pose_rotation_y(v_p.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf v_pose_rotation_z(v_p.yaw, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f v_pose_translation(v_p.x, v_p.y, v_p.z);
        Eigen::Matrix4f key_pose_transform =
                (v_pose_translation * v_pose_rotation_z * v_pose_rotation_y * v_pose_rotation_x).matrix();

        std::cout<<"key_pose_transform "<<key_pose_transform<<std::endl;

        ///transform local laser data to global data
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud( *newCloud, *tmp, key_pose_transform);
        tmp->height=1;
        tmp->width=tmp->size();
        tmp->points.resize(tmp->width*tmp->height);

        ///save global single frame laser data
        std::ostringstream global;
        global<<i;
        std::string global_path = save_loop_global_lidar_path_ ;
        std::string globalframe = global.str();
        globalframe.append("global.pcd");
        global_path.append(globalframe);
        pcl::io::savePCDFileBinary(global_path, *tmp );

        /*
        ///局部关键帧点云转换到全局坐标
        pcl::PointCloud<pcl::PointXYZI>::Ptr ori(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud( *newCloud, *ori,  pose_optimize.matrix());
        ori->height=1;
        ori->width=ori->size();
        ori->points.resize(ori->width*ori->height);

        std::string global_ori_path = save_ori_global_lidar_path_ ;
        global_ori_path.append(globalframe);
        ///存储
        pcl::io::savePCDFileBinary(global_ori_path, *ori );

        ori->clear() ;
        */
        tmp->clear() ;
        newCloud->clear() ;
    }

    ///save optimizing loop trajectory
    pcl::io::savePCDFile(save_loop_trajectory_path_, trajectory_ );
}

/////////////////////////////////////////////////////////////////////////
/**
 * 发布点云数据
 * \param  in_publisher---发布的对象
 * \param  in_cloud_to_publish_ptr---发布的点云数据
*/
void robo_loop::publish_cloud(const ros::Publisher *in_publisher,
                              pcl::PointCloud<pcl::PointXYZ> &in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = header_;
    cloud_msg.header.frame_id = "rslidar";
    in_publisher->publish(cloud_msg);
}






