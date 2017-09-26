//
// Created by wenchi on 17-6-19.
//

#ifndef PROJECT_POLES_MUTLISTABLE_H
#define PROJECT_POLES_MUTLISTABLE_H


#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "ros/package.h"
#include <dirent.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "stdio.h"
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>


#include "tinyxml.h"

#pragma comment(lib,"tinyxml.lib")


std::string read_detecting_xml ;//= "/home/zyy/slam_ndt/src/poles_map/config/poles_detect.xml" ;


#define  PI 3.1415926

struct poleslabel2{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cv::Point2f location;
    double time;
    int label;
};

struct poleslabel{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cv::Point3f center;
    int label;
};


class poles_mutlistable {
public:
    poles_mutlistable (ros::NodeHandle node,
    ros::NodeHandle private_nh);
    ~poles_mutlistable (){}
    //入口函数


    void gridCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void pushFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

    void clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                   float in_min_height, float in_max_height);

    void genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,cv::Mat& mat);

    cv::Point2i trans(cv::Point2f pt);

    void icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg);

    void extractTree(const cv::Mat label,cv::Mat& mat);

    void genClusters2(const cv::Mat label,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,std::vector<poleslabel>& poles);

    void euclideanDistance(std::vector<poleslabel>& poles,std::vector<pcl::PointCloud<pcl::PointXYZI> >& in_cluster);

    void matormat(const cv::Mat temp1,const cv::Mat temp2,cv::Mat& temp3);

    void publishpoles(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);

    void features(const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);

    void clusterstoFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> >in_cluster,
                          std::vector<poleslabel2>& curPoles,pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position);

    void curFramesassotopreFrame(std::vector<poleslabel2>& curPoles,
                                 std::vector<poleslabel2> prePoles,
                                 std::map<int,int>& asso,
                                 std::vector<cv::Point2f>& centers,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position);

    void showcylinder(std::vector<cv::Point2f> centers);


    void publishpoles_tree(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud);

    void loadpcd(std::string filename,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);


    std::vector<pcl::PointCloud<pcl::PointXYZI> >in_cluster;
    std::vector<poleslabel2> curPoles;
    std::vector<poleslabel2> prePoles;

    double timeStam;
    std::map<int,int> asso;
    std::vector<cv::Point2f> centers;
    std::vector<int> indices;

    std_msgs::Header robosense_header;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_poles;
    ros::Publisher marker_pub;
    ros::Publisher pub_poles_track;
    ros::Publisher pub_poles_position;

    //特征数据
    std::vector<float> feature;//30=1+6+3+20
    std::vector<float> Pointnum;//1
    std::vector<float> Cov_mat;//6
    std::vector<float> Local_pose;//3
    std::vector<float> Slice_mat;//10
    float cov_scalar;

    std::ofstream featureFile;

    std::vector<cv::Scalar> _colors;
    int colorsNum,countPoints;
    float countPoints_top,countPoints_bottom,scalarPoints;

    double gridMiH;
    double gridMaH;
    int dilation_size;
    cv::Mat grid,grid_2,grid_3,temp_1,temp_2;
    double gWidth,gHeight;
    double miniGrid;
    int gridW,gridH;

    int frames;

    float in_clip_min_height,in_clip_max_height;


    float in_clip_min_height_2,in_clip_max_height_2;



    ///////////////////////zyy////////////////////////

    std::string save_poles_txt_path ;
    std::string load_local_lidar_path ;
    std::string load_pose_txt_path ;

    int start_frame ;
    int end_frame ;
    int frame_step ;

    std::ofstream  save_poles_txt ;
    ///存储mapping位姿序列容器
    std::vector<std::vector<float> > pose_data;
    int cloud_frame ;
    int txt_line ;

    Eigen::Matrix4f key_pose_transform ;

    std::fstream& GotoLine(std::fstream& file, unsigned int num);
    std::vector<float> dataCB(std::fstream& file, int line);

    void read_xml();

};


#endif //PROJECT_POLES_MUTLISTABLE_H
