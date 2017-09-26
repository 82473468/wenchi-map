//
// Created by wenchi on 17-6-19.
//

#include "poles_mutlistable.h"
 

poles_mutlistable::poles_mutlistable(ros::NodeHandle node,
                               ros::NodeHandle private_nh) {

    ///读入配置文件路径
    private_nh.getParam("read_detecting_xml",read_detecting_xml);
    ///读入配置文件内参数
    read_xml();
    ///存储杆状特征地物位置信息
    save_poles_txt.open(save_poles_txt_path);

    std::string topic = private_nh.param("topic", std::string("rslidar_points"));
 
    pub_poles = node.advertise<sensor_msgs::PointCloud2>("rslidar_raw",1000000);

    pub_poles_position=node.advertise<sensor_msgs::PointCloud2>("poles_position",10);

    in_clip_min_height = -0.2;
    in_clip_max_height =0.5; 

    in_clip_min_height_2 = 0.8;
    in_clip_max_height_2 = 1.5;

    gWidth = private_nh.param("gWidth",float(50.));
    gHeight = private_nh.param("gHeight",float(50.));
    miniGrid = private_nh.param("miniGrid",float(0.15));

    gridMiH = -500;gridMaH = 500;
    dilation_size = 1;

    //计算栅格长宽
    gridH = gHeight / miniGrid; gridW = gWidth / miniGrid;
    //用于分割的栅格图
    grid = cv::Mat::zeros(gridH, gridW, CV_8UC1);

    grid_2 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

    grid_3 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

    temp_1 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

    temp_2 = cv::Mat::zeros(gridH, gridW, CV_8UC1);

    colorsNum = 200;

    generateColors(_colors,colorsNum);



    std::fstream file(load_pose_txt_path,std::ios_base::in);

    ///生成位姿序列容器
    if(file.is_open()) {
        int line_num =1;
        while (!file.eof()) {
            std::vector<float> data_line;
            data_line = dataCB(file, line_num);
            pose_data.push_back(data_line);
            std::cout<<data_line[0]<<"  "<<data_line[1]<<"  "<<data_line[2]<<"  "<<data_line[3]<<"  "<<data_line[4]<<
                     "  "<<data_line[5]<<"  "<<data_line[6]<<std::endl;
            line_num ++;
        }
    }

   int txt_line = 0 ;

    for(cloud_frame=start_frame;cloud_frame<end_frame;cloud_frame=cloud_frame+frame_step)
    {
        txt_line++;

        std::cout<<"cloud_frame "<<cloud_frame<<std::endl;
        std::cout<<"txt_line "<<txt_line<<std::endl;

        std::vector<float> data_line;
        data_line = pose_data.at(txt_line);
 
        std::ostringstream sstream;

        sstream << cloud_frame;

        std::string stime_add = load_local_lidar_path;
        std::string stime = sstream.str();
        stime.append("laser.pcd");
        stime_add.append(stime);

        pcl::PointCloud<pcl::PointXYZI> cloud_ori ;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

        pcl::io::loadPCDFile(stime_add, cloud_ori);

            pcl::PointXYZI p;
            double r;
            int min_scan_range = 1 ;
            for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = cloud_ori.begin();
                 item != cloud_ori.end(); item++) {
                p.x = (double) item->x;
                p.y = (double) item->y;
                p.z = (double) item->z;
                p.intensity = (double) item->intensity;

                r = p.x * p.x + p.y * p.y;
                if ((r > min_scan_range))
                {
                    cloud->points.push_back(p);
                }
            }
 
 
         Eigen::AngleAxisf v_pose_rotation_x(data_line[4], Eigen::Vector3f::UnitX());
         Eigen::AngleAxisf v_pose_rotation_y(data_line[5], Eigen::Vector3f::UnitY());
         Eigen::AngleAxisf v_pose_rotation_z(data_line[6], Eigen::Vector3f::UnitZ());

         Eigen::Translation3f v_pose_translation(data_line[1], data_line[2], data_line[3]);

         key_pose_transform =
                (v_pose_translation * v_pose_rotation_z * v_pose_rotation_y * v_pose_rotation_x).matrix();

         std::cout<<"key_pose_transform "<<key_pose_transform<<std::endl;

         gridCallback(cloud);
    }
 
}

//////////////////////////////////////////读入配置文件///////////////////////////////////////////////////////////////
void poles_mutlistable::read_xml()
{
    TiXmlDocument doc ;
    if(!doc.LoadFile(read_detecting_xml))
    {
        std::cout<<"error_xml"<<std::endl;
    }
    else
    {
        std::cout<<"read_xml"<<std::endl;
    }

    TiXmlElement* node = doc.FirstChildElement("POLES_DETECT") ;

    TiXmlElement* save_poles_txt_path_Elem = node->FirstChildElement("save_poles_txt_path") ;
    TiXmlElement* load_local_lidar_path_Elem = node->FirstChildElement("load_local_laser_path") ;
    TiXmlElement* load_pose_txt_path_Elem = node->FirstChildElement("load_pose_txt_path") ;
    TiXmlElement* start_frame_Elem = node->FirstChildElement("start_frame") ;
    TiXmlElement* end_frame_Elem = node->FirstChildElement("end_frame") ;
    TiXmlElement* frame_step_Elem = node->FirstChildElement("frame_step") ;


    ///存储检测到的z杆状地物位置信息的txt
    save_poles_txt_path = save_poles_txt_path_Elem->GetText();
    ///载入局部单帧点云数据的路径
    load_local_lidar_path = load_local_lidar_path_Elem->GetText();
    ///载入单帧数据对应位姿的txt
    load_pose_txt_path = load_pose_txt_path_Elem->GetText();
    ///地图起始帧
    start_frame = atoi(start_frame_Elem->GetText());
    ///地图终止帧
    end_frame = atoi(end_frame_Elem->GetText());
    ///地图间隔帧
    frame_step = atoi(frame_step_Elem->GetText());
}

std::fstream& poles_mutlistable::GotoLine(std::fstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(unsigned int p=0; p < num - 1; ++p){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

std::vector<float> poles_mutlistable::dataCB(std::fstream& file, int line)
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
 
void poles_mutlistable::publishpoles_tree(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud){
    sensor_msgs::PointCloud2 output_poles;
    pcl::toROSMsg(*poles_cloud,output_poles);
    output_poles.header.frame_id="base_link2";
    in_publisher->publish(output_poles);
}

void poles_mutlistable::gridCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // float theta=-(82./180)*PI;
    float theta=0;
    transform_1(0,0)=cosf(theta);
    transform_1(0,1)=-sinf(theta);
    transform_1(1,0)=sinf(theta);
    transform_1(1,1)=cosf(theta);
    pcl::transformPointCloud(*cloud,*cloud_2,transform_1);
  

    publishpoles_tree(&pub_poles, cloud_2);
    clock_t start=clock();
    pushFrame(cloud_2);
    clock_t end=clock();
    std::cout<<"the time is"<<end-start<<std::endl;
}

void poles_mutlistable::pushFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr_2 (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_ptr (new  pcl::PointCloud<pcl::PointXYZRGB>);

    //利用栅格法进行两个高度分割
    clipCloud(cloud,clipped_cloud_ptr,in_clip_min_height,in_clip_max_height);

    clipCloud(cloud,clipped_cloud_ptr_2,in_clip_min_height_2,in_clip_max_height_2);
 
    genGrid(clipped_cloud_ptr,grid);

    genGrid(clipped_cloud_ptr_2,grid_2);
     dilation_size=1;
    cv::Mat element = getStructuringElement( cv::MORPH_RECT,
                                         cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                         cv::Point( dilation_size, dilation_size ) );
    dilate(grid,grid,element);

    dilate(grid_2,grid_2,element);
 
    cv::Mat label;//int型
    icvprCcaBySeedFill(grid,label);

    cv::Mat label_2;
    icvprCcaBySeedFill(grid_2,label_2);
 
    //将柱状分割提取出来
    extractTree(label,temp_1);

    extractTree(label_2,temp_2);

    //将两个高度融合一起
    matormat(temp_1,temp_2,grid_3);
 
    cv::Mat label_3;
    icvprCcaBySeedFill(grid_3,label_3);
    //进一步判断，将柱状物体分割出来
    std::vector<poleslabel> poles;

    genClusters2(label_3,cloud,poles);
    poleslabel tmp_pole;
    tmp_pole.label=0;
    poles.push_back(tmp_pole);
    poles.push_back(tmp_pole);

    std::vector<pcl::PointCloud<pcl::PointXYZI> > in_cluster;
 
    euclideanDistance(poles, in_cluster);

    poles_position->clear();
    clusterstoFrames(in_cluster, curPoles,poles_position);
 

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_lidar(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud( *poles_position, *global_lidar, key_pose_transform);

     for(unsigned int i = 0; i<global_lidar->points.size();i++ )
    {
           pcl::PointXYZI p = global_lidar->points.at(i);
           save_poles_txt<<i<<"    "<<p.x<<"   "<<p.y<<"   "<<p.z<<"   "<<std::endl;
     }

    std::cout<<"poles_size = "<<global_lidar->points.size()<<std::endl;
     publishpoles(&pub_poles_position, poles_position);

}


 

void poles_mutlistable::clusterstoFrames(const std::vector<pcl::PointCloud<pcl::PointXYZI> > in_cluster,
                                    std::vector<poleslabel2>& curPoles,pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position){
    curPoles.clear();
    std::vector<pcl::PointCloud<pcl::PointXYZI> >::const_iterator it=in_cluster.begin();
    int tmp_label=0;
    for(;it!=in_cluster.end();++it)
    {
        float x_position=0,y_position=0;
        std::vector<cv::Point2f> points;
        for(unsigned int i=0;i<(*it).points.size();++i)
        {
            x_position+=(*it).points[i].x;
            y_position+=(*it).points[i].y;
            points.push_back(cv::Point2f((*it).points[i].x,(*it).points[i].y));
        }
        x_position/=(*it).points.size();
        y_position/=(*it).points.size();
        cv::RotatedRect rect=cv::minAreaRect(points);
        float long_size=rect.size.height>rect.size.width?rect.size.height:rect.size.width;
        float short_size=rect.size.height>rect.size.width?rect.size.width:rect.size.height;
        float len=sqrtf(x_position*x_position+y_position*y_position);
        float offset=(long_size-short_size)/2;
        x_position=x_position+offset*x_position/len;
        x_position=x_position+offset*x_position/len;
        poleslabel2 pole_tmp;
        pole_tmp.cloud=*it;
        pole_tmp.label=tmp_label++;
        pole_tmp.location=cv::Point2f(x_position,y_position);
        curPoles.push_back(pole_tmp);

        pcl::PointXYZI position_point;
        position_point.x=x_position;
        position_point.y=y_position;
        position_point.z=0;
        poles_position->points.push_back(position_point);

    }
}


bool compX(cv::Point2f pt1,cv::Point2f pt2){
    return pt1.x < pt2.x;
}

void poles_mutlistable::curFramesassotopreFrame(std::vector<poleslabel2>& curPoles,
                                                std::vector<poleslabel2> prePoles,
                                                std::map<int,int>& asso,
                                                std::vector<cv::Point2f>& centers,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position){
    asso.clear();
    centers.clear();
    std::vector<std::vector<cv::Point2f> > match;
    match.resize(curPoles.size());

    if(!prePoles.empty())
    {
        for(unsigned int i=0;i<curPoles.size();++i) {
            match[i].resize(prePoles.size());
            for (unsigned int j = 0; j < prePoles.size(); ++j) {
                float distance = sqrtf(pow(curPoles[i].location.x - prePoles[j].location.x, 2) +
                                       pow(curPoles[i].location.y - prePoles[j].location.y, 2));
                match[i][j].x = distance;
                match[i][j].y = j;
            }
            std::sort(match[i].begin(), match[i].end(), compX);
            if ((*match[i].begin()).x < 100) {
                centers.push_back(curPoles[i].location);
                pcl::PointXYZI point;
                point.x=curPoles[i].location.x;
                point.y=curPoles[i].location.y;
                point.z=0;
                poles_position->points.push_back(point);
            }
        }

    }
}

void poles_mutlistable::extractTree(const cv::Mat label,cv::Mat& mat){
    double min,max;
    minMaxIdx(label,&min,&max);
    std::vector<int> labelnum;
    labelnum.resize(max+1,0);
    mat.setTo(0);
    for(int i=0;i<label.rows;++i)
    {
        for(int j=0;j<label.cols;++j)
        {
            labelnum[label.at<int>(i,j)]++;
        }
    }

    for(int i=0;i<label.rows;++i)
    {
        for(int j=0;j<label.cols;++j)
        {
            if(labelnum[label.at<int>(i,j)]<30)
                mat.at<uchar>(i,j)=1;
        }
    }

}

void poles_mutlistable::euclideanDistance(std::vector<poleslabel>& poles,std::vector<pcl::PointCloud<pcl::PointXYZI> >& in_cluster){
    std::vector<poleslabel>::iterator it=poles.begin();
    for(;it!=poles.end()-1;++it)
    {
        std::vector<poleslabel>::iterator it_son=it+1;
        for(;it_son!=poles.end();++it_son)
        {
            if(it==it_son) continue;
            if((*it).label==0) continue;
            if((*it_son).label==0) continue;
            float distance=sqrtf(pow(((*it).center.x-(*it_son).center.x),2)+pow(((*it).center.y-(*it_son).center.y),2));
            if(distance<2)
            {
                (*it).label=0;
                (*it_son).label=0;
            }
        }
    }
    std::vector<poleslabel>::iterator it_son2=poles.begin();
    for(;it_son2!=poles.end();++it_son2)
    {
        if((*it_son2).label==1)
        {
            in_cluster.push_back((*it_son2).cloud);
        }
    }
}

void poles_mutlistable::genClusters2(const cv::Mat label,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, std::vector<poleslabel>& poles){
    double min,max;
    minMaxIdx(label,&min,&max);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > tmpPCDs;
    tmpPCDs.resize(max);

    std::vector<int> labelnum;
    labelnum.resize(max+1,0);
    for(int i=0;i<label.rows;++i)
    {
        for(int j=0;j<label.cols;++j)
        {
            labelnum[label.at<int>(i,j)]++;
        }
    }

    for (unsigned int i = 0; i < in_cloud_ptr->size(); ++i) {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        cv::Point2i tmpIndex = trans(cv::Point2f(tmpI.x,tmpI.y));
        if (tmpIndex.x >= 0 && tmpIndex.x < gridW && tmpIndex.y >= 0 && tmpIndex.y < gridH)
        {
            int index = label.at<int>(tmpIndex.y,tmpIndex.x);
            if (index <= 0)continue;
            if(labelnum[index]>40) continue;
            tmpPCDs[index - 1].push_back(tmpI);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr poles_posi(new pcl::PointCloud<pcl::PointXYZI>);
    poles_posi->clear();
    sensor_msgs::PointCloud2 output_poles;
    for (unsigned int i = 0; i < tmpPCDs.size(); ++i) {
        if(tmpPCDs[i].points.size()<10) continue;
        float z_min=999,z_max=-999;
        float x_pos=0,y_pos=0,z_pos=0;
        std::vector<cv::Point2f> points;
        points.clear();
        for(unsigned int j=0;j<tmpPCDs[i].points.size();++j)
        {
            if(tmpPCDs[i].points[j].z<z_min)
                z_min=tmpPCDs[i].points[j].z;
            if(tmpPCDs[i].points[j].z>z_max)
                z_max=tmpPCDs[i].points[j].z;
            x_pos+=tmpPCDs[i].points[j].x;
            y_pos+=tmpPCDs[i].points[j].y;
            z_pos+=tmpPCDs[i].points[j].z;
            points.push_back(cv::Point2f(tmpPCDs[i].points[j].x,tmpPCDs[i].points[j].y));
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud_=tmpPCDs[i];
        x_pos/=tmpPCDs[i].points.size();
        y_pos/=tmpPCDs[i].points.size();
        z_pos/=tmpPCDs[i].points.size();
        //********利用判定来识别柱子
        bool ignore=false;
        float distance=sqrtf(x_pos*x_pos+y_pos*y_pos);
        if(distance<8&&(z_max-z_min)<2)
            ignore=true;
        if(distance>8&&(z_max-z_min)<2.4)
            ignore=true;
        if(ignore) continue;
        features(cloud_);
        if(countPoints>1) continue;
         if(scalarPoints>3) continue;
        if(cov_scalar>5) continue;
        cv::RotatedRect rect=cv::minAreaRect(points);
        float long_size=rect.size.height>rect.size.width?rect.size.height:rect.size.width;
        float short_size=rect.size.height>rect.size.width?rect.size.width:rect.size.height;
         if(long_size>0.4&&long_size/short_size>2) continue;
         poleslabel pole;
        pole.cloud=tmpPCDs[i];
        pole.center=cv::Point3f(x_pos, y_pos, z_pos);
        pole.label=1;
        poles.push_back(pole);
    }
}


void poles_mutlistable::matormat(const cv::Mat temp1,const cv::Mat temp2,cv::Mat& temp3){
    temp3.setTo(0);
    for(int i=0;i<temp1.rows;++i)
    {
        for(int j=0;j<temp1.cols;++j)
        {
            if(temp1.at<uchar>(i,j)==1||temp2.at<uchar>(i,j)==1)
                temp3.at<uchar>(i,j)=1;
        }
    }
}


void poles_mutlistable::showcylinder(std::vector<cv::Point2f> centers){

    visualization_msgs::MarkerArray markers;
    markers.markers.clear();

    std::vector<cv::Point2f>::iterator it_son=centers.begin();
    int idx=0;
    for(;it_son!=centers.end();++it_son) {
        visualization_msgs::Marker marker;
        marker.header = robosense_header;
        marker.ns = "basic_shapes";
        marker.id =idx;
        marker.type = visualization_msgs::Marker::CYLINDER;

        marker.pose.position.x = (*it_son).x;
        marker.pose.position.y = (*it_son).y;
        marker.pose.position.z = 2.5;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 8;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5;

        marker.lifetime = ros::Duration(0.2);
        idx++;
        markers.markers.push_back(marker);
    }

    marker_pub.publish(markers);

}


void poles_mutlistable::clipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                               float in_min_height, float in_max_height){
    out_cloud_ptr->points.clear();
    for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
   
        if (in_cloud_ptr->points[i].z >= in_min_height &&
            in_cloud_ptr->points[i].z <= in_max_height)
        {
        
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}


void poles_mutlistable::genGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,cv::Mat& mat){
 
    mat.setTo(0);
  
    for (unsigned int i = 0; i < in_cloud_ptr->size(); ++i) {
        pcl::PointXYZI tmpI = in_cloud_ptr->points[i];
        cv::Point2i tmpIdx = trans(cv::Point2f(tmpI.x,tmpI.y));
        if(tmpIdx.x >= 0 && tmpIdx.x < gridW && tmpIdx.y >= 0 && tmpIdx.y < gridH){
            mat.at<uchar>(tmpIdx.y,tmpIdx.x)=1;
        }
    }
}

cv::Point2i poles_mutlistable::trans(cv::Point2f pt){
    return cv::Point2i(int(pt.x / miniGrid) + gridW / 2, int(pt.y / miniGrid) + gridH / 2);
}


void poles_mutlistable::icvprCcaBySeedFill(const cv::Mat& _binImg, cv::Mat& _lableImg){
    // connected component analysis (4-component)
    // use seed filling algorithm
    // 1. begin with a foreground pixel and push its foreground neighbors into a stack;
    // 2. pop the top pixel on the stack and label it with the same label until the stack is empty
    //
    // foreground pixel: _binImg(x,y) = 1
    // background pixel: _binImg(x,y) = 0

    if (_binImg.empty() ||
        _binImg.type() != CV_8UC1)
    {
        return ;
    }

    cv::Mat edge_binImg = cv::Mat::zeros(_binImg.rows + 2,_binImg.cols + 2,CV_8UC1);

    _binImg.copyTo(edge_binImg(cv::Rect(1,1,_binImg.cols,_binImg.rows)));

    _lableImg.release() ;
    edge_binImg.convertTo(_lableImg, CV_32SC1) ;

    int label = 1 ;  // start by 2

    int rows = _binImg.rows - 1 ;
    int cols = _binImg.cols - 1 ;

    for (int i = 1; i < rows-1; i++)
    {
        int* data= _lableImg.ptr<int>(i) ;
        for (int j = 1; j < cols-1; j++)
        {
            if (data[j] == 1)
            {
                std::stack<std::pair<int,int> > neighborPixels ;
                neighborPixels.push(std::pair<int,int>(i,j)) ;// pixel position: <i,j>


                ++label ;  // begin with a new label
                while (!neighborPixels.empty())
                {
                    // get the top pixel on the stack and label it with the same label
                    std::pair<int,int> curPixel = neighborPixels.top() ;
                    int curX = curPixel.first ;
                    int curY = curPixel.second ;
                    _lableImg.at<int>(curX, curY) = label ;

                    // pop the top pixel
                    neighborPixels.pop() ;

                    // push the 4-neighbors (foreground pixels)
                    if (_lableImg.at<int>(curX, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX, curY+1) == 1)
                    {// right pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY+1)) ;
                    }
                    if (_lableImg.at<int>(curX-1, curY) == 1)
                    {// up pixel
                        neighborPixels.push(std::pair<int,int>(curX-1, curY)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY) == 1)
                    {// down pixel
                        neighborPixels.push(std::pair<int,int>(curX+1, curY)) ;
                    }
                    /*
                    if (_lableImg.at<int>(curX-1, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY-1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX-1, curY+1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    if (_lableImg.at<int>(curX+1, curY+1) == 1)
                    {// left pixel
                        neighborPixels.push(std::pair<int,int>(curX, curY-1)) ;
                    }
                    */
                }
            }
        }
    }


    cv::Mat tmp = _lableImg.clone();
    _lableImg.release();
    _lableImg = cv::Mat::zeros(tmp.rows - 2,tmp.cols - 2,CV_32SC1);
    tmp(cv::Rect(1,1,_lableImg.cols,_lableImg.rows)).copyTo(_lableImg);
    tmp.release();

}


void poles_mutlistable::features(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_){

    //点数
    Pointnum.resize(1,0);
    Pointnum[0] = cloud_->points.size();
    //协方差
    Cov_mat.resize(6,0);
    float x_avr = 0;
    float y_avr = 0;
    float z_avr = 0;
    for(unsigned int pp = 0; pp < cloud_->points.size();++pp){
        x_avr += cloud_->points[pp].x;
        y_avr += cloud_->points[pp].y;
        z_avr += cloud_->points[pp].z;
    }
    x_avr /= cloud_->points.size();
    y_avr /= cloud_->points.size();
    z_avr /= cloud_->points.size();
    for(unsigned int pp = 0; pp < cloud_->points.size();++pp){
        Cov_mat[0] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].x - x_avr);//cov(x,x)
        Cov_mat[1] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].y - y_avr);//cov(y,y)
        Cov_mat[2] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].y - y_avr);//cov(x,y)
        Cov_mat[3] += (cloud_->points[pp].x - x_avr) * (cloud_->points[pp].z - z_avr);//cov(x,z)
        Cov_mat[4] += (cloud_->points[pp].y - y_avr) * (cloud_->points[pp].z - z_avr);//cov(y,z)
        Cov_mat[5] += (cloud_->points[pp].z - z_avr) * (cloud_->points[pp].z - z_avr);//cov(z,z)
    }
    for(int i = 0; i < 6; ++i){
        Cov_mat[i] /= (cloud_->points.size()-1);
    }
    float cov_min=999,cov_max=-999;
    for(int i=0;i<2;++i) {
        if (Cov_mat[i] < cov_min)
            cov_min = Cov_mat[i];
        if (Cov_mat[i] > cov_max)
            cov_max = Cov_mat[i];
    }
    cov_scalar=cov_max/cov_min;
    //切片
    float min_z = 100;
    float max_z = -100;
    for (unsigned int i = 0; i < cloud_->points.size(); i++)
    {
        if (cloud_->points[i].z < min_z)
            min_z = cloud_->points[i].z;
        if (cloud_->points[i].z > max_z)
            max_z = cloud_->points[i].z;
    }
    int sliceNum = 7;
    Slice_mat.resize(sliceNum*2,0);
    float sliceStep = (max_z - min_z) / sliceNum;
//    std::std::cout<<sliceStep<<std::std::std::endl;
    countPoints=0;
    countPoints_top=0;
    countPoints_bottom=0;
    scalarPoints=0;
    if (sliceStep>0.1) {
        std::vector<std::vector<pcl::PointXYZI> > sliceHistgram;
        sliceHistgram.resize(sliceNum);
        for (unsigned int i = 0; i < cloud_->points.size(); i++) {
            int sliceIndex = (cloud_->points[i].z - min_z) / sliceStep;
            if (sliceIndex == sliceNum)
                sliceIndex -= 1;
            sliceHistgram[sliceIndex].push_back(cloud_->points[i]);
        }

        for (unsigned int i = 0; i < sliceHistgram.size(); i++) {
            if (sliceHistgram[i].size() == 0) {
                countPoints++;
            }
            if (i < 3)
                countPoints_bottom += sliceHistgram[i].size();
            else
                countPoints_top += sliceHistgram[i].size();
        }
        scalarPoints=countPoints_top/countPoints_bottom;
    }

//    std::cout<<"4"<<std::std::endl;
    feature.clear();
//    feature.insert(feature.end(),Pointnum.begin(),Pointnum.end());
    feature.push_back(Pointnum[0]);
    feature.push_back((max_z - min_z));
    feature.push_back(countPoints);
//    feature.insert(feature.end(),Cov_mat.begin(),Cov_mat.end());
//    feature.insert(feature.end(),Local_pose.begin(),Local_pose.end());
//    feature.insert(feature.end(),Slice_mat.begin(),Slice_mat.end());
}

void poles_mutlistable::publishpoles(const ros::Publisher* in_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr poles_cloud){
    sensor_msgs::PointCloud2 output_poles;
    pcl::toROSMsg(*poles_cloud,output_poles);
    output_poles.header=robosense_header;
    in_publisher->publish(output_poles);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "segpoles");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    poles_mutlistable poles(node,private_nh);

    ros::spin();

    return 0;
}
