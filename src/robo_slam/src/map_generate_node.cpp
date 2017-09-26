
/** @ brief map generate ; downsample ; rotate
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 */

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "tinyxml.h"


///global string variable:read config xml file path
std::string g_read_xml_path ;
///global TiXmlDocument variable: xml doc
TiXmlDocument g_doc ;
///global enum variable :process_type_name
enum g_process_type{MAP_SAVE, VOXEL_MAP, MAP_ROTATE};
///global string variable :process_type_name
std::string g_process_type_string[3]={"MAP_SAVE","VOXEL_MAP","MAP_ROTATE"};
///global int variable:the type of processing
int g_process_name ;

////////////////////////////////////////////////////////////////////////////

///global string variable : load_global_pcd_path for map_save function
std::string g_map_save_load_global_pcd_path ;
///global string variable : global_name for map_save function
std::string g_map_save_global_name ;
///global string variable : global_map_path for map_save function
std::string g_map_save_save_global_map_path ;
///global int variable : start_frame for map_save function
int g_map_save_start_frame ;
///global int variable : end_frame for map_save function
int g_map_save_end_frame ;
///global int variable : frame_step for map_save function
int g_map_save_frame_step ;

/** @brief generate global map by accumulate
*/
void map_save()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);

    for(int i = g_map_save_start_frame ; i < g_map_save_end_frame ; i = i + g_map_save_frame_step)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr global(new pcl::PointCloud<pcl::PointXYZI>);

        std::ostringstream global_num;
        global_num<<i;
        std::string global_file= g_map_save_load_global_pcd_path ;
        std::string global_frame= global_num.str();
        global_frame.append(g_map_save_global_name);
        global_file.append(global_frame);

        pcl::io::loadPCDFile(global_file,  *global);

        std::cout<< i <<std::endl;
        if(global->points.size()>0)
        {
            *map+=*global ;
        }
    }
    ///save global map
    std::ostringstream start_num;
    start_num<<g_map_save_start_frame;
    std::string map_start = start_num.str();
    std::ostringstream end_num;
    end_num<<g_map_save_end_frame;
    std::string map_end = end_num.str();
    std::string  map_save_path = g_map_save_save_global_map_path ;

    map_start.append("_");
    map_start.append(map_end);
    map_start.append("_");
    map_start.append("map.pcd");
    map_save_path.append(map_start);

    pcl::io::savePCDFileASCII(map_save_path, *map );

    std::cout<<map_save_path<<std::endl;
    std::cout<<"map is saved."<<std::endl;

}

////////////////////////////////////////////////////////////////////////////////

///global string variable : load_map_path_pcd for voxel_map function
std::string g_voxel_load_map_path_pcd ;
///global string variable : save_map_path_pcd for voxel_map function
std::string g_voxel_save_map_path_pcd ;
///global string variable : downsampling_voxel_leaf_size for voxel_map function
float g_voxel_leaf_size ;

/** @brief downsample map by voxel function
*/
void voxel_map()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ori(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile( g_voxel_load_map_path_pcd,  *map_ori);

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(g_voxel_leaf_size, g_voxel_leaf_size, g_voxel_leaf_size);
    voxel_grid_filter.setInputCloud(map_ori);
    voxel_grid_filter.filter(*voxel_filtered_map);
    pcl::io::savePCDFileBinary( g_voxel_save_map_path_pcd, *voxel_filtered_map );

    std::cout<<"voxel map_ is saved."<<std::endl;

}
//////////////////////////////////////////////////////////////////////////////////////////

///global string variable : load_global_map for map_rotate function
std::string g_map_rotate_load_global_map ;
///global string variable : save_global_map for map_rotate function
std::string g_map_rotate_save_global_map ;
///global float variable : map_rotate_parameters for map_rotate function
float g_map_rotate_t_x, g_map_rotate_t_y, g_map_rotate_t_z, g_map_rotate_t_roll, g_map_rotate_t_pitch, g_map_rotate_t_yaw ;

/** @brief rotate map by transformPointCloud function
*/
void map_rotate()
{

    pcl::PointCloud<pcl::PointXYZI> local_cloud_ori;
    pcl::io::loadPCDFile(g_map_rotate_load_global_map, local_cloud_ori);

    ///generate transform matrix according to map_rotate_parameters
    Eigen::AngleAxisf current_rotation_x(g_map_rotate_t_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf current_rotation_y(g_map_rotate_t_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf current_rotation_z(g_map_rotate_t_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f current_translation(g_map_rotate_t_x, g_map_rotate_t_y, g_map_rotate_t_z);

    Eigen::Matrix4f tf_ndt_local_to_global =
            (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(local_cloud_ori, *transformed_scan_ptr, tf_ndt_local_to_global);

    pcl::io::savePCDFileBinary(g_map_rotate_save_global_map, *transformed_scan_ptr);
    std::cout<<"rotate_map is saved "<<std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////

/** @brief read config xml for process_type
*/
void read_xml_process_type()
{

    if(!g_doc.LoadFile(g_read_xml_path))
    {
        std::cout<<"error_xml"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = g_doc.FirstChildElement("GENERATING") ;
    TiXmlElement* node_process = node->FirstChildElement("process_type_name");

    TiXmlElement* process_type_name_Elem = node_process->FirstChildElement("process_type_name");
    std::string process_type_name =  process_type_name_Elem->GetText() ;
    std::cout<<"process_type_name = "<<process_type_name<<std::endl;

    if(process_type_name == g_process_type_string[0])
    {
        g_process_name = MAP_SAVE;
    }
    else
    {
        if(process_type_name == g_process_type_string[1])
        {
            g_process_name = VOXEL_MAP;
        }
        else
        {
            g_process_name = MAP_ROTATE;
        }
    }
 }

/** @brief read config xml for map_save function
*/
void read_map_save_xml()
{
    TiXmlElement* node = g_doc.FirstChildElement("GENERATING") ;
    TiXmlElement* node_process = node->FirstChildElement("MAP_SAVE");

    TiXmlElement* map_save_load_global_pcd_path_Elem = node_process->FirstChildElement("g_map_save_load_global_pcd_path");
    TiXmlElement* map_save_global_name_Elem = node_process->FirstChildElement("g_map_save_global_name");
    TiXmlElement* map_save_save_global_map_path_Elem = node_process->FirstChildElement("g_map_save_save_global_map_path");
    TiXmlElement* map_save_start_frame_Elem = node_process->FirstChildElement("g_map_save_start_frame");
    TiXmlElement* map_save_end_frame_Elem = node_process->FirstChildElement("g_map_save_end_frame");
    TiXmlElement* map_save_frame_step_Elem = node_process->FirstChildElement("g_map_save_frame_step");

    g_map_save_load_global_pcd_path =  map_save_load_global_pcd_path_Elem->GetText() ;
    g_map_save_global_name =  map_save_global_name_Elem->GetText() ;
    g_map_save_save_global_map_path = map_save_save_global_map_path_Elem->GetText();
    g_map_save_start_frame =  atoi(map_save_start_frame_Elem->GetText());
    g_map_save_end_frame =  atoi(map_save_end_frame_Elem->GetText()) ;
    g_map_save_frame_step = atoi(map_save_frame_step_Elem->GetText()) ;
}

/** @brief read config xml for voxel_map function
*/
void read_voxel_map_xml()
{
    TiXmlElement* node = g_doc.FirstChildElement("GENERATING") ;
    TiXmlElement* node_process = node->FirstChildElement("VOXEL_MAP");

    TiXmlElement* voxel_load_map_path_pcd_Elem = node_process->FirstChildElement("g_voxel_load_map_path_pcd");
    TiXmlElement* voxel_save_map_path_pcd_Elem = node_process->FirstChildElement("g_voxel_save_map_path_pcd");
    TiXmlElement* voxel_leaf_size_Elem = node_process->FirstChildElement("g_voxel_leaf_size");

    g_voxel_load_map_path_pcd =  voxel_load_map_path_pcd_Elem->GetText() ;
    g_voxel_save_map_path_pcd =  voxel_save_map_path_pcd_Elem->GetText() ;
    g_voxel_leaf_size =  atof(voxel_leaf_size_Elem->GetText());
}

/** @brief read config xml for map_rotate function
*/
void read_map_rotate_xml()
{
    TiXmlElement* node = g_doc.FirstChildElement("GENERATING") ;
    TiXmlElement* node_process = node->FirstChildElement("MAP_ROTATE");

    TiXmlElement* map_rotate_load_global_map_Elem = node_process->FirstChildElement("g_map_rotate_load_global_map");
    TiXmlElement* map_rotate_save_global_map_Elem = node_process->FirstChildElement("g_map_rotate_save_global_map");
    TiXmlElement* map_rotate_t_x_Elem = node_process->FirstChildElement("g_map_rotate_t_x");
    TiXmlElement* map_rotate_t_y_Elem = node_process->FirstChildElement("g_map_rotate_t_y");
    TiXmlElement* map_rotate_t_z_Elem = node_process->FirstChildElement("g_map_rotate_t_z");
    TiXmlElement* map_rotate_t_roll_Elem = node_process->FirstChildElement("g_map_rotate_t_roll");
    TiXmlElement* map_rotate_t_pitch_Elem = node_process->FirstChildElement("g_map_rotate_t_pitch");
    TiXmlElement* map_rotate_t_yaw_Elem = node_process->FirstChildElement("g_map_rotate_t_yaw");

    g_map_rotate_load_global_map =  map_rotate_load_global_map_Elem->GetText() ;
    g_map_rotate_save_global_map =  map_rotate_save_global_map_Elem->GetText() ;
    g_map_rotate_t_x = atoi(map_rotate_t_x_Elem->GetText()) ;
    g_map_rotate_t_y = atoi(map_rotate_t_y_Elem->GetText()) ;
    g_map_rotate_t_z = atoi(map_rotate_t_z_Elem->GetText()) ;
    g_map_rotate_t_roll = atoi(map_rotate_t_roll_Elem->GetText()) ;
    g_map_rotate_t_pitch = atoi(map_rotate_t_pitch_Elem->GetText()) ;
    g_map_rotate_t_yaw = atoi(map_rotate_t_yaw_Elem->GetText()) ;

}

/** @brief main function
*/
int main(int argc, char** argv)
{
    ros::init(argc,argv,"robo_map_generating");

    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("g_read_xml_path",g_read_xml_path);
    read_xml_process_type();

    std::cout<<"g_process_name = "<<g_process_name<<std::endl;

    switch (g_process_name)
    {
        case MAP_SAVE:
            read_map_save_xml();
            std::cout<<"process_type = map_save"<<std::endl;
            map_save();
            break;
        case VOXEL_MAP:
            read_voxel_map_xml();
            std::cout<<"process_type = voxel_map"<<std::endl;
            voxel_map();
            break;
        case MAP_ROTATE:
            read_map_rotate_xml();
            std::cout<<"process_type = map_rotate"<<std::endl;
            map_rotate();
            break;
        default:
            break;
    }

    return 0;
}

