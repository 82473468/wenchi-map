/** @ brief generate global poles feature map
 *  Author zyy
 *	Copyright (C) 2017, Robosense
 *
 *  License: It is a commercial and not open source licensed product developed by Suteng Innovation Technology Co.,Ltd
 *  https://www.robosense.ai
 *  商业软件,未经许可，不得使用
 *
 */

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include "tinyxml.h"

std::string g_read_mapping_xml ;

unsigned int g_line_num = 1;
unsigned int g_tree_id = 1;
std::string g_load_poles_txt ;
std::string g_save_poles_pcd ;
float g_merge_dis ;
unsigned int g_poles_detect_times ;

struct PoleFeature{
    /// mean of multiply measurements.currently using only 2D position, but eventually the third dimension could be used
    pcl::PointXYZ position;
    /// number of times the same feature been detected
    long int times = 0;
    /// unique identity number of the pole
    long int id = 0;
    std::vector<pcl::PointXYZ> historyPoints;
} ;

std::vector<PoleFeature> g_mappedFeatures;

//////////////////////////////////////////////////////////////////////////////////////////

/** @brief read config xml for mapping
*/
void read_xml()
{
    TiXmlDocument doc ;
    if(!doc.LoadFile(g_read_mapping_xml))
    {
        std::cout<<"error_xml"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("POLES_MAP") ;

    TiXmlElement* load_poles_txt_Elem = node->FirstChildElement("load_poles_txt") ;
    TiXmlElement* save_poles_pcd_Elem = node->FirstChildElement("save_poles_pcd") ;
    TiXmlElement* merge_dis_Elem = node->FirstChildElement("merge_dis") ;
    TiXmlElement* poles_detect_times_Elem = node->FirstChildElement("poles_detect_times") ;

    ///load global position information (x,y,z) of all the poles
    g_load_poles_txt = load_poles_txt_Elem->GetText();
    ///save the final global pole feature map
    g_save_poles_pcd = save_poles_pcd_Elem->GetText();
    ///the min merging distance threshold
    g_merge_dis = atof(merge_dis_Elem->GetText());
    ///the threshold of detected times of single pole
    g_poles_detect_times = atoi(poles_detect_times_Elem->GetText());
 }
/////////////////////////////////////////////////////////////////////////

/**@brief read current line txt data
  * @param[in] file  txt data
  * @param[in] num   line index
 */
std::fstream& go_to_line(std::fstream &file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(unsigned int p=0; p < (num - 1); ++p)
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
std::vector<double> get_txt_data(std::fstream &file, int line)
{
    go_to_line(file, line);
    std::string str;
    std::getline(file, str);
    std::vector<double> data;

    std::istringstream iss(str);
    int count;
    for (count=0; count<4; count++)
    {
        std::string sub;
        iss >> sub;
        double value = ::atof(sub.c_str());
        data.push_back(value);
    }

    return data;
}

/////////////////////////////////////////////////////////////////////////

/**@brief add single frame pole to global map
  * @param[in] tree_current  single frame pole point
 */
void addCurrentTree(pcl::PointXYZ &tree_current)
{
        std::cout<<"addCurrentTree"<<std::endl;
 
        PoleFeature current_tree ;
        current_tree.position.x = tree_current.x;
        current_tree.position.y = tree_current.y;
        current_tree.position.z = tree_current.z;

        int total = g_mappedFeatures.size()-1;
        bool exst =false;

        std::cout<<"total ="<<total<<std::endl;

        while(total>-1)
        {
            PoleFeature map_tree = g_mappedFeatures[total] ;

            float disX = fabsf( map_tree.position.x - current_tree.position.x);
            float disY = fabsf( map_tree.position.y - current_tree.position.y);
            ///update the information of existed pole feature
            if(disX <g_merge_dis&&disY<g_merge_dis)
            {
                g_mappedFeatures[total].historyPoints.push_back(current_tree.position);

                float NewTime = g_mappedFeatures[total].times + 1 ;
                float LastWeight = (float)(NewTime-1)/NewTime ;
                float NewWeight = 1.0 - LastWeight ;
 
                ///update the hit times
                g_mappedFeatures[total].times ++ ;
                ///update the average position
                g_mappedFeatures[total].position.x = LastWeight*map_tree.position.x + NewWeight*current_tree.position.x ;
                g_mappedFeatures[total].position.y = LastWeight*map_tree.position.y + NewWeight*current_tree.position.y ;
                g_mappedFeatures[total].position.z = LastWeight*map_tree.position.z + NewWeight*current_tree.position.z ;
                 exst = true ;
            }/// one time update finished
            total-- ;
        }

        if(exst == false)///add new feature point
        {
            current_tree.times ++ ;
            current_tree.id = g_tree_id;
            ///add new feature point
            g_mappedFeatures.push_back(current_tree) ;
            g_tree_id ++;
            std::cout<<"newPoint"<<std::endl;
        }

    std::cout<<"total ="<<g_mappedFeatures.size()<<std::endl;

}///addCurrentTree

/////////////////////////////////////////////////////////////////////

/** @brief main function
*/
int main(int argc, char** argv)
{

    ros::init(argc,argv,"poles_map_generate");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("read_mapping_xml_",g_read_mapping_xml);
    read_xml();

    std::fstream file(g_load_poles_txt,std::ios_base::in);
    std::vector<double>  data;
    if(file.is_open())
    {
        while(!file.eof())
        {
            std::vector<double>  data_line;
            data_line = get_txt_data(file, g_line_num);
            std::cout<<"line "<<g_line_num<<std::endl;
            std::cout<<"data1 "<<data_line[0]<<std::endl;
            std::cout<<"data2 "<<data_line[1]<<std::endl;
            std::cout<<"data3 "<<data_line[2]<<std::endl;
            std::cout<<"data4 "<<data_line[3]<<std::endl;

            for(unsigned int i = 0; i<data_line.size();i++)
            {
                pcl::PointXYZ tree_p;
                tree_p.x=data_line[1];
                tree_p.y=data_line[2];
                tree_p.z=data_line[3];

                addCurrentTree(tree_p);
            }
            g_line_num++;
        }
 
        pcl::PointCloud<pcl::PointXYZ> tree_points_save ;
 
        for (float i = 0; i < g_mappedFeatures.size(); i++)
        {
            if(g_mappedFeatures[i].times>g_poles_detect_times)
            {
                 pcl::PointXYZ p;
                 p.x = g_mappedFeatures[i].position.x;
                 p.y = g_mappedFeatures[i].position.y;
                 p.z = g_mappedFeatures[i].position.z;
 
                 tree_points_save.push_back(p);
            }
        }
        pcl::io::savePCDFile(g_save_poles_pcd, tree_points_save);
    }
    else
    {
        std::cout<<"wrong "<<std::endl;
    }

    return 1;
}
