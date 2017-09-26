//
// Created by zyy on 17-7-11.
//


#include "robo_mapping.h"

int main(int argc, char** argv)
{

    ros::init(argc,argv,"robo_mapping");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
    Robosense::robo_map mapping(node,private_nh);
    ros::Rate rate(100);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
