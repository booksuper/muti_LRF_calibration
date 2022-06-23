#pragma once
#include "oriData2OptimData.h"

/*
 *Author : ShuDengdeng
 *Email  : 2237380450@qq.com
 *功能：主要包含一些工具函数，比如分割直线，lserscan转点云等
 * 2022.623
*/

//将opencv的直线格式转为Tline_2D格式
vector<pair<size_t,TLine2D_My> >  vec4f2Tline(vector<Vec4f> &input_lines);

//将点云分割成两条线，以opencv的格式存储
void seg_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc,vector<vector<Point2f> > & lines_points);


//laserscan转点云
void laser2pc(const sensor_msgs::LaserScan::ConstPtr &p,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_,string frame);

//显示原始点云和拟合的直线
#if 0
void view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<Vec4f> lines)
  { 
    cout<<"共找到   "<<lines.size()<<"  条直线!!!!!!!!"<<endl;
    
    viewer.setBackgroundColor(0.5, 0.5, 0.5, 0);
    viewer.addCoordinateSystem(0.3);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color (cloud,"x");
    viewer.addPointCloud(cloud,color, "pc");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc"); 
    
    
    for(int i=0; i<lines.size(); i++)
    {
        float k = lines[i][1] / lines[i][0];//斜率
        float b = lines[i][3] - k * lines[i][2];//截距
        pcl::PointXYZ p1(2,2*k+b, 0);
        pcl::PointXYZ p2(-2, -2*k+b, 0);
        cout<<"直线     "<<i+1<<"   上两点坐标： "<<p1<<",  "<<p2<<endl;
        cout<<"直线     "<<i+1<<"   方程： "<<"y="<<k<<"x"<<"+"<<b<<endl;
        viewer.addLine(p1, p2, 240-40*i, 0, 0, "line"+to_string(i), 0);
    }
    
    
    viewer.spinOnce();
    
  }
#endif