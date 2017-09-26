/** @ brief saving offline single frame lidar data
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int g_frame_num = 1;///lidar counter

std::string g_save_local_lidar_path ;
std::string g_save_local_lidar_path_folder ;

/** @brief save offline single frame lidar data
    @param[in] input the single frame lidar data(point cloud)
*/
static void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_laser(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*input, *local_laser);
    std::cout << " g_frame_num "<< g_frame_num << std::endl;

    std::ostringstream local_num;
    local_num << g_frame_num;
    std::string save_local_path = g_save_local_lidar_path_folder;
    std::string local_frame_num = local_num.str();
    local_frame_num.append("laser.pcd");
    save_local_path.append(local_frame_num);
    ///save single frame lidar data
    pcl::io::savePCDFileBinary(save_local_path, *local_laser );

    ++g_frame_num;
}

/**@brief create folder (if the folder does not exist)
 * @param[in] directory_path  the folder path
*/
void if_mkdir(std::string & directory_path)
{
    char *_file = (char*)directory_path.c_str();
    if (!fopen(_file,"wb"))
    {
        std::cout<<"mkdir"<<std::endl;
        std::cout<<_file<<std::endl;
        mkdir(_file,S_IRWXU);
    }
}

/**@brief main function subscribe lidar data
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "robo_lidar_save");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Rate rate(10);

    private_nh.getParam("g_save_local_lidar_path",g_save_local_lidar_path);
    g_save_local_lidar_path_folder = g_save_local_lidar_path;
    g_save_local_lidar_path_folder.append("local_laser/");
    ///if the folder doex not exist, create it
    if_mkdir(g_save_local_lidar_path_folder);

    while(ros::ok())
    {
        ros::Subscriber points_sub = nh.subscribe("rslidar_points", 100000, points_callback);
        ros::spin();
        rate.sleep();
    }

    return 0;
}