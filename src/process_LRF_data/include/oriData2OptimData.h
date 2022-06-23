#pragma once
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>
//这五个头文件将laserscan转为pointcloud
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
//这四个头文件都是为了实时同步地获得多个传感器数据
#include <message_filters/synchronizer.h>//同步器
#include<message_filters/time_synchronizer.h>//时间同步戳
#include<message_filters/subscriber.h>//消息过滤中的订阅对象
#include <message_filters/sync_policies/approximate_time.h>//近似的时间同步戳

using namespace cv;
using namespace std;
using namespace Eigen;
/*
 *Author : ShuDengdeng
 *Email  : 2237380450@qq.com
 *功能：将拟合的直线方程转为优化器需要的数据
 *2022.623
*/


//分割后2维直线的数据结构
//它存储线段的中心和一个方向向量，以及两者的协方差。
struct line2D_obs
{
    Matrix<double,2,1> center; // The center of the observed line segment
    Matrix<double,2,2> cov_center;//线段中心的协方差
    Matrix<double,2,1> dir; // A unitary direction vector
    Matrix<double,2,2> cov_dir;//方向向量协方差
};

//三维直线数据结构
//通过将二维直线和lrf相对世界坐标系的初始矩阵结合得到三维直线，暂时不需要三维的
#if 0
struct line_3D
{
    
    //整个函数功能就是将二维直线的坐标和LRF姿态矩阵结合得到三维直线的数据
    void get_3D_params(const line2D_obs &line_2D, const CPose3D &LRF_pose)
    {
        // center = LRF_pose.getRotationMatrix()*line_2D.center + LRF_pose.m_coords;
        //截取LRF的旋转矩阵的3x2,与直线的中心相乘，得到3x1的列向量
        center_rot = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.center;
        center = center_rot + LRF_pose.m_coords;
        cov_center = LRF_pose.getRotationMatrix().block(0,0,3,2)* line_2D.cov_center * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
        dir = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.dir;
        cov_dir = LRF_pose.getRotationMatrix().block(0,0,3,2) * line_2D.cov_dir * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
    };
    //三维直线中心
    Matrix<double,3,1> center; 
    Matrix<double,3,1> center_rot; 
    Matrix<double,3,3> cov_cente
    Matrix<double,3,1> dir; 
    Matrix<double,3,3> cov_dir;
};
typedef vector<vector<line_3D> > CO_3D;
#endif

//LRF观测的两条直线
struct CO_1
{
    
    unsigned id_LRF;
    line2D_obs lines_co[2];
    
};

//两个LRF观测的两对直线
typedef vector<CO_1> CO; 

typedef Matrix<double,50,1> CO_vector;


//存储分割后的直线方程ax+by+c=0，三个系数分别是abc
struct TLine2D_My
{
    double coefs[3];
};

//co转为ransac需要的向量形式
//一条二维直线有12个参数，一个co有四条二维直线，就有48个参数，外加两个lrf ID号，共50
CO_vector CO2vector(const CO &co);

//将co_vector的向量形式转回co
CO vector2CO(const CO_vector &co_vector);

//生成cos数据
void generate_cos(int M_num_LRFs,vector<vector<pair<size_t,TLine2D_My> > > detected_lines,vector<CO> &vCOs);