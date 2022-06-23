#include "data_util.h"

/*
 *Author : ShuDengdeng
 *Email  : 2237380450@qq.com
 *功能：主要包含一些工具函数，比如分割直线，lserscan转点云等
 *2022.623
*/

//将opencv的直线格式转为Tline_2D格式
vector<pair<size_t,TLine2D_My> >  vec4f2Tline(vector<Vec4f> &input_lines)
{
    vector<pair<size_t,TLine2D_My> >  out_res;
    //初始化大小
    out_res.resize(2);
    for(int i = 0;i<input_lines.size();i++)
    {
        double a1 = input_lines[i][1] / input_lines[i][0];
        double b1 = -1;
        double c1 = input_lines[i][3] - input_lines[i][2] * a1;
        //double cof[3] = {a1,b1,c1};
        out_res[i].first = i;
        out_res[i].second.coefs[0] = a1;
        out_res[i].second.coefs[1] = b1;
        out_res[i].second.coefs[2] = c1;


    }
    return out_res;
}

//将点云分割成两条线，以opencv的格式存储
void seg_lines(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc,vector<vector<Point2f> > & lines_points)
{
    float max_x = 0;
    int max_x_indice = 0;
    float seg = 0;
    //找到x最大值的点索引
    for(int i=0;i<input_pc->points.size();i++)
    {
        if(max_x <= input_pc->points[i].x)
        {
            max_x = input_pc->points[i].x;
            max_x_indice = i;
        }
    }
    seg = input_pc->points[max_x_indice].y;
    //以x最大的点作为分割点，小于分割点的y值是一条线，大于是另一条线
    Point2f p1(0,0);
    Point2f p2(0,0);
    for(int i=0;i<input_pc->points.size();i++)
    {
        if(input_pc->points[i].y <= seg)
        { 
             p1.x = input_pc->points[i].x;
             p1.y = input_pc->points[i].y;
             lines_points[0].push_back(p1);
        }
        else
        {
            p2.x = input_pc->points[i].x;
            p2.y = input_pc->points[i].y;
            lines_points[1].push_back(p2);
            
        }
    }
}


//laserscan转点云
void laser2pc(const sensor_msgs::LaserScan::ConstPtr &p,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_,string frame)
{
    laser_geometry::LaserProjection laser_pro;
    tf::TransformListener tfListener;    
    
    //这个对象在后面转点云的时候要传给LaserProjection对象
    
    //这一句话在kinect版本已经弃用
    //tfListener.setExtrapolationLimit(ros::Duration(0.1));
   
    sensor_msgs::PointCloud2 cloud2;
    //laser转为pointcloud2,第一个参数是发布的话题的名称，一定要严格对应，不然会报错
    //base_camera_link_0  scan_up_raw
    laser_pro.transformLaserScanToPointCloud(frame,*p,cloud2,tfListener);
    //pointcloud2转为pointcloud
    pcl::fromROSMsg(cloud2,*cloud_);
}
