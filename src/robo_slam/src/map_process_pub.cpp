//
// Created by zyy on 17-7-29.
//

#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <stack>
#include <vector>
#include <fstream>

#include <ros/ros.h>
#include <opencv/cv.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <boost/thread/thread.hpp>



////////////////////////////读取当前行的文件内容//////////////////////////////
std::fstream& GotoLine(std::fstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(int p=0; p < num - 1; ++p){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}
////////////////////////////读取当前行的文件内容//////////////////////////////
std::vector<float> dataCB(std::fstream& file, int line)
{

    GotoLine(file, line);
    std::string str;
    std::getline(file, str);
    std::vector<float> data;

    std::istringstream iss(str);
    int count;

    for (count=0; count<7; count++)
    {

        std::string sub;

        iss >> sub;

        double value = ::atof(sub.c_str());
        data.push_back(value);

    }

    return data;

}

/////////////////////////////////////////////////发布点云数据///////////////////////////////////////////////////
void publishCloud(const ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZI> & in_cloud_to_publish)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(in_cloud_to_publish, cloud_msg);
    cloud_msg.header.frame_id= "rslidar";
    in_publisher->publish(cloud_msg);
}


void byd()
{

    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map_", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;


    pcl::io::loadPCDFile("/media/zyy/新加卷/byd/0728_rotate/572_3910_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷/byd/0725/local_laser/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/byd/0728_rotate/save_loop_pose_0728.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 8000 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

            data_line[4] = data_line[4] - 0.11 ;
            pose_data.push_back(data_line);

            line_num ++;
        }
    }

    publishCloud(&pub_map, map);

    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i=i+2)
    {
        ///生成递增局部地图路径
        std::ostringstream local ;

        local << i;

        std::string local_path = load_local_lidar_path;

        std::string localframe = local.str();
        localframe.append("laser.pcd");
        local_path.append(localframe);

        pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;

        pcl::io::loadPCDFile(local_path, local_cloud_ori);

        int txt_line = i/2;
        std::vector<float> data_pose = pose_data.at(txt_line);

        ///由mapping位姿生成旋转矩阵
        Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
        Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

        Eigen::Matrix4f tf_ndt_local_to_global =
                (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

        pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(local_cloud_ori, transformed_scan, tf_ndt_local_to_global);

        ///发布当前轨迹
        publishCloud(&pub_lidar, transformed_scan);

        pcl::PointXYZI p;
        p.x = data_pose[1] ;
        p.y = data_pose[2] ;
        p.z = data_pose[3] ;
        p.intensity = 200 ;

        odom.push_back(p);

        publishCloud(&pub_odom, odom);

        for(int k = 0;k<2;k++)
        {
            std::cout<<k<<std::endl;
        }

    }



        ros::spin();
        rate.sleep();
    }

}




void jd( )
{


    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map_", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;//(new pcl::PointCloud<pcl::PointXYZI>());


    pcl::io::loadPCDFile("/media/zyy/新加卷/jd/1_7520_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷2/jd/jd_lidar_local_2/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/jd/0724/save_loop_pose_0724.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 13000 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

          //  data_line[4] = data_line[4] - 0.11 ;
            pose_data.push_back(data_line);

            line_num ++;
        }
    }

    publishCloud(&pub_map, map);

    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i=i+2)
        {
            ///生成递增局部地图路径
            std::ostringstream local ;

            local << i;

            std::string local_path = load_local_lidar_path;

            std::string localframe = local.str();
            localframe.append("laser.pcd");
            local_path.append(localframe);

            pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;

            pcl::io::loadPCDFile(local_path, local_cloud_ori);

            std::vector<float> data_pose = pose_data.at(i);

            ///由mapping位姿生成旋转矩阵
            Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
            Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

            Eigen::Matrix4f tf_ndt_local_to_global =
                    (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

            pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(local_cloud_ori, transformed_scan, tf_ndt_local_to_global);

            ///发布当前轨迹
            publishCloud(&pub_lidar, transformed_scan);

            pcl::PointXYZI p;
            p.x = data_pose[1] ;
            p.y = data_pose[2] ;
            p.z = data_pose[3] ;
            p.intensity = 200 ;

            odom.push_back(p);

            publishCloud(&pub_odom, odom);

            for(int k = 0;k<1;k++)
            {
                std::cout<<k<<std::endl;
            }



        }



        ros::spin();
        rate.sleep();
    }

}


void jd1( )
{


    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map_", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;//(new pcl::PointCloud<pcl::PointXYZI>());


    pcl::io::loadPCDFile("/media/zyy/新加卷/jd/1_7520_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷2/jd/jd_lidar_local_2/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/jd/0724/save_loop_pose_0724.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 13000 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

            //  data_line[4] = data_line[4] - 0.11 ;
            pose_data.push_back(data_line);

            line_num ++;
        }
    }


    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i=i+2)
        {
            ///生成递增局部地图路径
            std::ostringstream local ;

            local << i;

            std::string local_path = load_local_lidar_path;

            std::string localframe = local.str();
            localframe.append("laser.pcd");
            local_path.append(localframe);

            pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;

            pcl::io::loadPCDFile(local_path, local_cloud_ori);

            std::vector<float> data_pose = pose_data.at(i);

            ///由mapping位姿生成旋转矩阵
            Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
            Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

            Eigen::Matrix4f tf_ndt_local_to_global =
                    (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

            pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(local_cloud_ori, transformed_scan, tf_ndt_local_to_global);

            ///发布当前lidar
            publishCloud(&pub_lidar, local_cloud_ori);

            pcl::PointXYZI p;
            p.x = data_pose[1] ;
            p.y = data_pose[2] ;
            p.z = data_pose[3] ;
            p.intensity = 200 ;

            odom.push_back(p);


            Eigen::Matrix4f global_inv = tf_ndt_local_to_global.inverse();

            pcl::PointCloud<pcl::PointXYZI> transformed_odom ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(odom, transformed_odom, global_inv);

            publishCloud(&pub_odom, transformed_odom);

            pcl::PointCloud<pcl::PointXYZI> transformed_map ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(map, transformed_map, global_inv);

            publishCloud(&pub_map, transformed_map);

        }



        ros::spin();
        rate.sleep();
    }

}

void gyy( )
{

    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map_", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;//(new pcl::PointCloud<pcl::PointXYZI>());


    pcl::io::loadPCDFile("/media/zyy/新加卷/工业园/1_4185_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷2/data_93/data_93_local_ng/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/工业园/save_loop_pose_0728.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 4150 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

            //  data_line[4] = data_line[4] - 0.11 ;
            pose_data.push_back(data_line);

            line_num ++;
        }
    }

    publishCloud(&pub_map, map);

    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i++)
        {
            ///生成递增局部地图路径
            std::ostringstream local ;

            local << i;

            std::string local_path = load_local_lidar_path;

            std::string localframe = local.str();
            localframe.append("laser.pcd");
            local_path.append(localframe);

            pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;

            pcl::io::loadPCDFile(local_path, local_cloud_ori);

            std::vector<float> data_pose = pose_data.at(i);

            ///由mapping位姿生成旋转矩阵
            Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
            Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

            Eigen::Matrix4f tf_ndt_local_to_global =
                    (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

            pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(local_cloud_ori, transformed_scan, tf_ndt_local_to_global);

            ///发布当前轨迹
            publishCloud(&pub_lidar, transformed_scan);

            pcl::PointXYZI p;
            p.x = data_pose[1] ;
            p.y = data_pose[2] ;
            p.z = data_pose[3] ;
            p.intensity = 200 ;

            odom.push_back(p);

            publishCloud(&pub_odom, odom);

            for(int k = 0;k<1;k++)
            {
                std::cout<<k<<std::endl;
            }



        }

        ros::spin();
        rate.sleep();
    }

}


void gyy1( )
{


    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory_", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map_", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;//(new pcl::PointCloud<pcl::PointXYZI>());



    pcl::io::loadPCDFile("/media/zyy/新加卷/工业园/lm_93/1_4185_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷2/data_93/data_93_local_2/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/工业园/lm_93/save_loop_pose_0728.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 4150 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

            data_line[3] = 0 ;
            pose_data.push_back(data_line);

            line_num ++;
        }
    }


    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i=i+2)
        {
            ///生成递增局部地图路径
            std::ostringstream local ;

            local << i;

            std::string local_path = load_local_lidar_path;

            std::string localframe = local.str();
            localframe.append("laser.pcd");
            local_path.append(localframe);

            pcl::PointCloud<pcl::PointXYZI> local_cloud_ori ;

            pcl::io::loadPCDFile(local_path, local_cloud_ori);

            std::vector<float> data_pose = pose_data.at(i);

            ///由mapping位姿生成旋转矩阵
            Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
            Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

            Eigen::Matrix4f tf_ndt_local_to_global =
                    (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

            pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(local_cloud_ori, transformed_scan, tf_ndt_local_to_global);

            ///发布当前lidar
            publishCloud(&pub_lidar, local_cloud_ori);

            pcl::PointXYZI p;
            p.x = data_pose[1] ;
            p.y = data_pose[2] ;
            p.z = data_pose[3] ;
            p.intensity = 200 ;

            odom.push_back(p);


            Eigen::Matrix4f global_inv = tf_ndt_local_to_global.inverse();

            pcl::PointCloud<pcl::PointXYZI> transformed_odom ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(odom, transformed_odom, global_inv);

            publishCloud(&pub_odom, transformed_odom);

            pcl::PointCloud<pcl::PointXYZI> transformed_map ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(map, transformed_map, global_inv);

            publishCloud(&pub_map, transformed_map);

        }



        ros::spin();
        rate.sleep();
    }

}

void rs( )
{

    ros::NodeHandle node;

    ros::Rate rate(100);

    ///发布mapping地图轨迹
    ros::Publisher pub_lidar = node.advertise<sensor_msgs::PointCloud2>("rslidar_points", 1);
    ///发布mapping地图轨迹
    ros::Publisher pub_odom = node.advertise<sensor_msgs::PointCloud2>("trajectory", 1);

    ///发布mapping地图轨迹
    ros::Publisher pub_map = node.advertise<sensor_msgs::PointCloud2>("map", 1);



    pcl::PointCloud<pcl::PointXYZI> map ;//(new pcl::PointCloud<pcl::PointXYZI>());


    pcl::io::loadPCDFile("/media/zyy/新加卷/rs_32/mapping_32_5/mapping_0905/1_2460_map_voxel.pcd", map);

    std::cout<<map.size()<<std::endl;

    std::string load_local_lidar_path = "/media/zyy/新加卷/rs_32/mapping_32_5/local_laser/";


    ///读入mapping位姿信息
    std::fstream file("/media/zyy/新加卷/rs_32/mapping_32_5/mapping_0905/save_loop_pose.txt",std::ios_base::in);

    int start_frame = 1 ;
    int end_frame = 2460 ;

    std::vector<std::vector<float> > pose_data ;
    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (line_num<end_frame) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);

            pose_data.push_back(data_line);

            line_num ++;
        }
    }



    while(ros::ok()){


        pcl::PointCloud<pcl::PointXYZI> odom ;


        for(int i = start_frame;i<end_frame;i++)
        {
            publishCloud(&pub_map, map);
            ///生成递增局部地图路径
            std::ostringstream local ;

            local << i;

            std::string local_path = load_local_lidar_path;

            std::string localframe = local.str();
            localframe.append("laser.pcd");
            local_path.append(localframe);

            pcl::PointCloud<pcl::PointXYZI> local_cloud  ;

            pcl::io::loadPCDFile(local_path, local_cloud);

            ///filter NAN points
            pcl::PointCloud<pcl::PointXYZI> local_cloud_ori;
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
                if ((r > 1)&&(p.z>-0.5)&&(p.z<1))
                {
                    local_cloud.push_back(p);
                }
            }

            std::vector<float> data_pose = pose_data.at(i);

            ///由mapping位姿生成旋转矩阵
            Eigen::AngleAxisf current_rotation_x( data_pose[4], Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf current_rotation_y( data_pose[5], Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf current_rotation_z( data_pose[6], Eigen::Vector3f::UnitZ());
            Eigen::Translation3f current_translation(data_pose[1], data_pose[2], data_pose[3]);

            Eigen::Matrix4f tf_ndt_local_to_global =
                    (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

            pcl::PointCloud<pcl::PointXYZI> transformed_scan ;//(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(local_cloud, transformed_scan, tf_ndt_local_to_global);

            ///发布当前轨迹
            publishCloud(&pub_lidar, transformed_scan);

            pcl::PointXYZI p_odom;
            p_odom.x = data_pose[1] ;
            p_odom.y = data_pose[2] ;
            p_odom.z = data_pose[3] ;
            p_odom.intensity = 200 ;

            odom.push_back(p_odom);

            publishCloud(&pub_odom, odom);

            for(int k = 0;k<25000;k++)
            {
                std::cout<<k<<std::endl;
            }



        }

        ros::spin();
        rate.sleep();
    }

}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "map_process_pub");

//    byd();

//    jd1();

//    gyy1();
    rs();
    return 0;
}
