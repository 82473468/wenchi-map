/** @ brief robo config xml file generate
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "tinyxml.h"

std::string g_save_xml_path = "/home/zyy/slam_ndt/src/robo_slam/config/" ;

int g_type = 2;

void robo_mapping_xml()
{
    std::string mapxml = "robo_mapping.xml";
    std::string map_path = g_save_xml_path ;
    map_path.append(mapxml);

    TiXmlDocument doc;
    TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);

    TiXmlElement* firstLevel=new TiXmlElement("MAPPING");


    const char* AttributeName[12]={"load_local_laser_path_","save_mapping_path_",
                                  "start_frame_","end_frame_","frame_step_","submap_num_",
                                   "min_scan_range_",
                                   "ndt_max_iter_","ndt_cell_size_res_","ndt_step_size_","ndt_trans_eps_",};

    ///载入离线单帧局部点云数据路径
    std::string load_local_lidar_path = "/media/zyy/新加卷/byd_2/0725/local_laser/";
    ///存储单帧全局点云数据路径
    std::string save_mapping_path = "/media/zyy/新加卷/byd_2/0807/";
    ///起始帧,终止帧
    std::string start_frame = "1";
    std::string end_frame = "14310";
    ///输入地图采样间隔
    std::string frame_step = "1";
    ///历史子地图帧数
    std::string submap_num = "400";
    ///输入点云滤波范围
    std::string min_scan_range = "1";
    ///ndt配准参数--最大迭代次数
    std::string ndt_max_iter = "30";
    ///ndt配准参数--栅格化尺寸,
    std::string ndt_cell_size_res = "1";
    ///ndt配准参数--迭代步长
    std::string ndt_step_size = "0.1";
    ///ndt配准参数--容忍误差
    std::string ndt_trans_eps = "0.01";

    std::vector<std::string> value;
    value.push_back(load_local_lidar_path);
    value.push_back(save_mapping_path);
    value.push_back(start_frame);
    value.push_back(end_frame);
    value.push_back(frame_step);
    value.push_back(submap_num);
    value.push_back(min_scan_range);
    value.push_back(ndt_max_iter);
    value.push_back(ndt_cell_size_res);
    value.push_back(ndt_step_size);
    value.push_back(ndt_trans_eps);

    for(int i = 0;i<value.size();i++)
    {
        TiXmlElement* secondLevel = new TiXmlElement(AttributeName[i]);
        secondLevel->LinkEndChild(new TiXmlText(value.at(i)));
        firstLevel->LinkEndChild(secondLevel);
    }

    doc.LinkEndChild(firstLevel);
    doc.SaveFile(map_path);
}

void robo_loopping_xml()
{
    std::string mapxml = "robo_loopping.xml";
    std::string map_path = g_save_xml_path ;
    map_path.append(mapxml);

    TiXmlDocument doc;
    TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);

    TiXmlElement* firstLevel=new TiXmlElement("LOOPPING");


    const char* AttributeName[19]={"load_local_lidar_path_","save_loop_global_path_",
                                   "start_frame_","end_frame_","frame_step_",
                                   "map_z_ratio_",
                                   "min_scan_range_",
                                   "ndt_max_iter_","ndt_cell_size_res_","ndt_step_size_","ndt_trans_eps_",
                                   "loop_begin_","loop_step_","loop_submap_num_","loop_success_step_","loop_search_range_",
                                   "check_loop_min_dis_","loop_success_score_","loop_z_ratio_"};

    ///载入离线单帧局部点云数据路径
    std::string load_local_lidar_path = "/media/zyy/新加卷/byd_2/0725/local_laser/";
    ///存储单帧全局点云数据路径
    std::string save_loop_global_path = "/media/zyy/新加卷/byd_2/0807/";

    ///起始帧,终止帧
    std::string start_frame = "1";
    std::string end_frame = "14309";
    ///输入地图采样间隔
    std::string frame_step = "1";
    ///制图z缩放系数
    std::string map_z_ratio = "0.5";

    ///输入点云滤波范围
    std::string min_scan_range = "1";

    ///ndt配准参数--最大迭代次数
    std::string ndt_max_iter = "50";
    ///ndt配准参数--栅格化尺寸,
    std::string ndt_cell_size_res = "0.5";
    ///ndt配准参数--迭代步长
    std::string ndt_step_size = "0.1";
    ///ndt配准参数--容忍误差
    std::string ndt_trans_eps = "0.01";

    ///开始回环
    std::string loop_begin = "7000";
    ///回环间隔
    std::string loop_step = "30";
    ///历史关键帧子地图帧数
    std::string loop_submap_num = "30";
    ///回环成功,加大间隔搜索
    std::string loop_success_step = "100";
    ///回环搜索范围帧
    std::string loop_search_range = "6000";
    ///回环搜索距离阈值
    std::string check_loop_min_dis = "40000";
    ///回环成功匹配分数阈值
    std::string loop_success_score = "0.1";
    ///闭环优化后z方向缩放比例
    std::string loop_z_ratio = "0";


    std::vector<std::string> value;
    value.push_back(load_local_lidar_path);
    value.push_back(save_loop_global_path);
    value.push_back(start_frame);
    value.push_back(end_frame);
    value.push_back(frame_step);
    value.push_back(map_z_ratio);
    value.push_back(min_scan_range);
    value.push_back(ndt_max_iter);
    value.push_back(ndt_cell_size_res);
    value.push_back(ndt_step_size);
    value.push_back(ndt_trans_eps);
    value.push_back(loop_begin);
    value.push_back(loop_step);
    value.push_back(loop_submap_num);
    value.push_back(loop_success_step);
    value.push_back(loop_search_range);
    value.push_back(check_loop_min_dis);
    value.push_back(loop_success_score);
    value.push_back(loop_z_ratio);

    for(int i = 0;i<value.size();i++)
    {
        TiXmlElement* secondLevel = new TiXmlElement(AttributeName[i]);
        secondLevel->LinkEndChild(new TiXmlText(value.at(i)));
        firstLevel->LinkEndChild(secondLevel);
    }

    doc.LinkEndChild(firstLevel);
    doc.SaveFile(map_path);
}

void robo_map_generate()
{
    enum process_type{VOXEL_MAP , MAP_SAVE , MAP_ROTATE};

    std::string mapxml = "robo_map_generating.xml";
    std::string map_path = g_save_xml_path ;
    map_path.append(mapxml);

    TiXmlDocument doc;
    TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "", "");
    doc.LinkEndChild(decl);

    /////////////////////////////////////////////////////////////////////////////////////

    TiXmlElement* firstLevel = new TiXmlElement("GENERATING");

    const char* type_name[4]={"process_type_name","MAP_SAVE", "VOXEL_MAP", "MAP_ROTATE" };

    TiXmlElement* secondLevel_0 = new TiXmlElement(type_name[0]);
    std::string process_type_name = "process_type::MAP_ROTATE";
    TiXmlElement* thirdLevel = new TiXmlElement(type_name[0]);
    thirdLevel->LinkEndChild(new TiXmlText(process_type_name));

    firstLevel->LinkEndChild(secondLevel_0);

    ///////////////////////////////////////////////////////////////////////////////////////

    TiXmlElement* secondLevel_1 = new TiXmlElement(type_name[1]);

    const char* map_save_para[6]={"g_map_save_load_global_pcd_path","g_map_save_global_name", "g_map_save_save_global_map_path",
                                  "g_map_save_start_frame" ,"g_map_save_end_frame", "g_map_save_frame_step"};

    ///载入全局坐标系下单帧点云数据
    std::string map_save_load_global_pcd_path  = "/media/zyy/新加卷1/byd/0728_rotate/loop_global/";
    ///全局坐标系下单帧点云数据名称
    std::string map_save_global_name  = "global.pcd";
    ///存储全局坐标系下完整地图数据
    std::string map_save_save_global_map_path = "/media/zyy/新加卷1/byd/0728_rotate/";
    ///地图起始帧
    std::string  map_save_start_frame = "1";
    ///地图结束帧
    std::string  map_save_end_frame = "7800";
    ///地图存储间隔
    std::string  map_save_frame_step = "15";

    std::vector<std::string> map_save_para_value ;
    map_save_para_value.push_back(map_save_load_global_pcd_path);
    map_save_para_value.push_back(map_save_global_name);
    map_save_para_value.push_back(map_save_save_global_map_path);
    map_save_para_value.push_back(map_save_start_frame);
    map_save_para_value.push_back(map_save_end_frame);
    map_save_para_value.push_back(map_save_frame_step);

    for(int i = 0;i<map_save_para_value.size();i++)
    {
        TiXmlElement* thirdLevel = new TiXmlElement(map_save_para[i]);
        thirdLevel->LinkEndChild(new TiXmlText(map_save_para_value.at(i)));
        secondLevel_1->LinkEndChild(thirdLevel);
    }

    firstLevel->LinkEndChild(secondLevel_1);

    /////////////////////////////////////////////////////////////////////////////////////////

    TiXmlElement* secondLevel_2 = new TiXmlElement(type_name[2]);

    const char* voxel_map_para[3]={"g_voxel_load_map_path_pcd","g_voxel_save_map_path_pcd", "g_voxel_leaf_size"};

    ///载入地图路径_pcd
    std::string voxel_load_map_path_pcd = "/media/zyy/新加卷/工业园/1_4185_map.pcd" ;
    ///存储降采样地图路径_pcd
    std::string voxel_save_map_path_pcd = "/media/zyy/新加卷/工业园/1_4185_map_voxel.pcd" ;
    ///降采样尺寸
    std::string voxel_leaf_size = "0.5" ;

    std::vector<std::string> voxel_map_para_value ;
    voxel_map_para_value.push_back(voxel_load_map_path_pcd);
    voxel_map_para_value.push_back(voxel_save_map_path_pcd);
    voxel_map_para_value.push_back(voxel_leaf_size);

    for(int i = 0;i<voxel_map_para_value.size();i++)
    {
        TiXmlElement* thirdLevel = new TiXmlElement(voxel_map_para[i]);
        thirdLevel->LinkEndChild(new TiXmlText(voxel_map_para_value.at(i)));
        secondLevel_2->LinkEndChild(thirdLevel);
    }

    secondLevel_2->LinkEndChild(new TiXmlText(process_type_name));
    firstLevel->LinkEndChild(secondLevel_2);

    ////////////////////////////////////////////////////////////////////////////////////////////

    TiXmlElement* secondLevel_3 = new TiXmlElement(type_name[3]);

    const char* map_rotate_para[8]={"g_map_rotate_load_global_map","g_map_rotate_save_global_map",
                                  "g_map_rotate_t_x", "g_map_rotate_t_y", "g_map_rotate_t_z",
                                  "g_map_rotate_t_roll" ,"g_map_rotate_t_pitch", "g_map_rotate_t_yaw"};

    ///载入全局坐标系下待旋转地图
    std::string map_rotate_load_global_map  =  "/media/zyy/新加卷/工业园/1_4185_map_voxel.pcd" ;
    ///存储全局坐标系下旋转后地图
    std::string map_rotate_save_global_map  =  "/media/zyy/新加卷/工业园/1_4185_map_voxel_rotate.pcd" ;

    std::string  map_rotate_t_x     = "0" ;
    std::string  map_rotate_t_y     = "0" ;
    std::string  map_rotate_t_z     = "0" ;
    std::string  map_rotate_t_roll  = "0" ;
    std::string  map_rotate_t_pitch = "0" ;
    std::string  map_rotate_t_yaw   = "0" ;

    std::vector<std::string> map_rotate_para_value ;
    map_rotate_para_value.push_back(map_rotate_load_global_map);
    map_rotate_para_value.push_back(map_rotate_save_global_map);
    map_rotate_para_value.push_back(map_rotate_t_x);
    map_rotate_para_value.push_back(map_rotate_t_y);
    map_rotate_para_value.push_back(map_rotate_t_z);
    map_rotate_para_value.push_back(map_rotate_t_roll);
    map_rotate_para_value.push_back(map_rotate_t_pitch);
    map_rotate_para_value.push_back(map_rotate_t_yaw);

    for(int i = 0;i<map_rotate_para_value.size();i++)
    {
        TiXmlElement* thirdLevel = new TiXmlElement(map_rotate_para[i]);
        thirdLevel->LinkEndChild(new TiXmlText(map_rotate_para_value.at(i)));
        secondLevel_3->LinkEndChild(thirdLevel);
    }

    firstLevel->LinkEndChild(secondLevel_3);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    doc.LinkEndChild(firstLevel);
    doc.SaveFile(map_path);
}


int main()
{

    switch (g_type)
    {
        case 1 :
            robo_mapping_xml() ;
            break;
        case 2:
            robo_loopping_xml() ;
            break ;
        case 3:
            robo_map_generate() ;
            break;
        default:
            break;
    }

    return 1;
}