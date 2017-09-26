//
// Created by zyy on 17-8-3.
//


#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <stack>
#include <vector>
#include <fstream>

#include <opencv/cv.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


struct scan_Data
{

    int point_num;
    float minRange;
    float maxRange;
    float sectionLength;

    std::vector<int> sectionCountVector;
    std::vector<float> sectionRatioVector;
    std::vector<float> rangeOfPointVector;
};


float section_num = 30 ;

float calculateRange(pcl::PointXYZI & p)
{
   // return pow( (pow(p.x,2)+pow(p.y,2)+pow(p.z,2)),0.5);
    return p.z;
}

int judgeBucket(float & range,float & minRange,float sectionLength)
{
    for (int i = 0; i < section_num ; ++i) {

        if((range>=(minRange+i*sectionLength))&&(range<=(minRange+(i+1)*sectionLength)))
        {
            return i;
        }

    }

    return section_num-1;
}


void histogram_generate(struct scan_Data & scan , pcl::PointCloud<pcl::PointXYZI>::Ptr scan_point)
{
    scan.point_num = scan_point->points.size();
    scan.maxRange = 0;
    scan.minRange = 9999;

    for (int i = 0; i < scan.point_num ; ++i) {

        float range_of_point = calculateRange(scan_point->points.at(i));
        scan.rangeOfPointVector.push_back(range_of_point);
        if(range_of_point>scan.maxRange)
        {
            scan.maxRange = range_of_point;
        }
        if(range_of_point<scan.minRange)
        {
            scan.minRange = range_of_point;
        }
    }

    scan.sectionLength = (scan.maxRange-scan.minRange)/section_num;

    for (int j = 0; j < section_num; ++j)
    {
        scan.sectionCountVector.push_back(0);
    }

    for (int k = 0; k <scan.point_num ; ++k) {

        int index = judgeBucket(scan.rangeOfPointVector.at(k),scan.minRange,scan.sectionLength);
        scan.sectionCountVector.at(index)++;
    }

    for (int l = 0; l < section_num ; ++l) {

        scan.sectionRatioVector.push_back((float)scan.sectionCountVector.at(l)/(float)scan.point_num);
    }

    std::cout<<"point_num = "<<scan.point_num<<std::endl;
    std::cout<<"min = "<<scan.minRange<<std::endl;
    std::cout<<"max = "<<scan.maxRange<<std::endl;
    std::cout<<"length = "<<scan.sectionLength<<std::endl;

    for (int l = 0; l < section_num ; ++l) {

        std::cout<<"sectionRatioVector = "<< scan.sectionRatioVector.at(l)<<std::endl;

    }


}

void histogram_w_dis(struct scan_Data & scan1,struct scan_Data & scan2)
{
    float EMD = 0;

    for (int i = 0; i <section_num ; ++i) {

        for (int j = 0; j <= i; ++j) {

            EMD +=fabs(scan1.sectionRatioVector.at(j)-scan2.sectionRatioVector.at(j));

        }

    }
    EMD /=section_num;

    std::cout<<"EMD = "<<EMD<<std::endl;

}


int main()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan1_point_ori(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan2_point_ori(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan1_point(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan2_point(new pcl::PointCloud<pcl::PointXYZI>());

    std::string load_scan1_path ="/media/zyy/新加卷/工业园/0811/local_laser/1523laser.pcd";
    std::string load_scan2_path ="/media/zyy/新加卷/工业园/0811/local_laser/6875laser.pcd";

    pcl::io::loadPCDFile(load_scan1_path, *scan1_point_ori);
    pcl::io::loadPCDFile(load_scan2_path, *scan2_point_ori);

    for (int i = 0; i <scan1_point_ori->points.size() ; ++i) {

        float dis = abs(scan1_point_ori->points.at(i).x)+abs(scan1_point_ori->points.at(i).y);

        if(dis>0.5)
        {
            scan1_point->points.push_back(scan1_point_ori->points.at(i));
        }

    }


    for (int i = 0; i <scan2_point_ori->points.size() ; ++i) {

        float dis = abs(scan2_point_ori->points.at(i).x)+abs(scan2_point_ori->points.at(i).y);

        if(dis>0.5)
        {
            scan2_point->points.push_back(scan2_point_ori->points.at(i));
        }

    }

    struct scan_Data scan1,scan2;

    histogram_generate(scan1,scan1_point);

    histogram_generate(scan2,scan2_point);

    histogram_w_dis(scan1,scan2);



    return  1 ;
}