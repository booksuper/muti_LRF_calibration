#include "data_util.h"
#include "oriData2OptimData.h"

#define SHOW_SEGMENTATION 1

/*
 *Author : ShuDengdeng
 *Email  : 2237380450@qq.com
 *功能：主函数
 *2022.623
*/

//初始化可视化对象
#if SHOW_SEGMENTATION
pcl::visualization::PCLVisualizer viewer("Viewer");
#endif

//存储LRF1,即scandown分割后的直线点
vector<vector<Point2f> > scandown_lines_point;
//存储LRF1分割后的直线系数
vector<Vec4f> scandown_lines_coeff;
//存储LRF2，即scanraw分割后的直线点和对应的直线系数
vector<vector<Point2f> > scanraw_lines_point;
vector<Vec4f> scanraw_lines_coeff;
//存储LRF3，即scanupraw分割后的直线点和对应的直线系数
vector<vector<Point2f> > scanupraw_lines_point;
vector<Vec4f> scanupraw_lines_coeff;
//直线的斜率，和上面的对应
float k,b,k2,b2,k3,b3;
//是否结束调用rosspin函数的标志
bool flag = true;
//lrf数量
int M_num_LRFs = 3;
//下采样系数，每隔10次采样一个数据	
const int decimation = 10;
//拟合直线的距离阈值	      
const double threshold_line	= 0.01;
//最小内点直线个数	    
const size_t min_inliers_line = 100;	  
//LRF索引
set<unsigned> idx_estim_LRFs;

vector<string> LRF_labels(M_num_LRFs);
//存储所有检测到的直线
vector<vector<pair<size_t,TLine2D_My> > > detected_lines;
//存错其中一个lrf检测的直线

//存co
vector<CO> vCOs;

int f = 0;

//回调函数
void process(const sensor_msgs::LaserScan::ConstPtr & p,const sensor_msgs::LaserScan::ConstPtr & p2,const sensor_msgs::LaserScan::ConstPtr & p3)
{
    //将laserscan转为点云向量
  pcl::PointCloud<pcl::PointXYZ>::Ptr scandown_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanraw_pc(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr scanupraw_pc(new pcl::PointCloud<pcl::PointXYZ>());
  laser2pc(p,scanraw_pc,"base_laser_link");
  laser2pc(p2,scandown_pc,"base_camera_link_ldlidar");
  laser2pc(p3,scanupraw_pc,"base_camera_link_0");
    
  //scandown的直线
  scandown_lines_point.clear();
  scandown_lines_coeff.clear();
  scandown_lines_point.resize(2);
  scandown_lines_coeff.resize(2);
  //scanraw的直线
  scanraw_lines_point.clear();
  scanraw_lines_coeff.clear();
  scanraw_lines_point.resize(2);
  scanraw_lines_coeff.resize(2);
  //scanupraw的直线
  scanupraw_lines_point.clear();
  scanupraw_lines_coeff.clear();
  scanupraw_lines_point.resize(2);
  scanupraw_lines_coeff.resize(2);
  //分割scandown的直线
  seg_lines(scandown_pc,scandown_lines_point);
  //分割scanraw的直线
  seg_lines(scanraw_pc,scanraw_lines_point);
  //分割scanupraw的直线
  seg_lines(scanupraw_pc,scanupraw_lines_point);

  for(int i = 0;i<2;i++)
  {
      fitLine(scandown_lines_point[i],scandown_lines_coeff[i],CV_DIST_L2,0,0.01,0.01);
      fitLine(scanraw_lines_point[i],scanraw_lines_coeff[i],CV_DIST_L2,0,0.01,0.01);
      fitLine(scanupraw_lines_point[i],scanupraw_lines_coeff[i],CV_DIST_L2,0,0.01,0.01);

  }   
//显示分割结果，可选
#if SHOW_SEGMENTATION
   //
  cout<<"共找到   "<<scandown_lines_coeff.size() + scanraw_lines_coeff.size() + scanupraw_lines_coeff.size()<<"  条直线!!!!!!!!"<<endl;
    
  viewer.setBackgroundColor(0.5, 0.5, 0.5, 0);
  viewer.addCoordinateSystem(0.3);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_down (scandown_pc,"x");
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_raw (scanraw_pc,"x");
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> color_upraw (scanupraw_pc,"x");
  viewer.addPointCloud(scandown_pc,color_down, "pc");
  viewer.addPointCloud(scanraw_pc,color_raw, "pc2");
  viewer.addPointCloud(scanupraw_pc,color_upraw, "pc3");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc"); 
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc2"); 
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc3"); 
    
  for(int i=0; i<scandown_lines_coeff.size(); i++)
    {
       //scandown
        k = scandown_lines_coeff[i][1] / scandown_lines_coeff[i][0];//斜率
        b = scandown_lines_coeff[i][3] - k * scandown_lines_coeff[i][2];//截距
        //scanraw
        k2 = scanraw_lines_coeff[i][1] / scanraw_lines_coeff[i][0];//斜率
        b2 = scanraw_lines_coeff[i][3] - k2 * scanraw_lines_coeff[i][2];//截距
         //scanupraw
        k3 = scanupraw_lines_coeff[i][1] / scanupraw_lines_coeff[i][0];//斜率
        b3 = scanupraw_lines_coeff[i][3] - k3 * scanupraw_lines_coeff[i][2];//截距
        //scandown直线的终末两点
        pcl::PointXYZ p1(2,2*k+b, 0);
        pcl::PointXYZ p2(-2, -2*k+b, 0);
        //scanraw直线的终末两点
        pcl::PointXYZ p3(0.8,0.8*k2+b2, 0);
        pcl::PointXYZ p4(-2, -2*k2+b2, 0);
        //scanupraw直线的终末两点
        pcl::PointXYZ p5(1.2,0.8*k3+b3, 0);
        pcl::PointXYZ p6(-2, -2*k3+b3, 0);
        cout<<"scandown直线     "<<i+1<<"   上两点坐标： "<<p1<<",  "<<p2<<endl;
        cout<<"scandown直线     "<<i+1<<"   方程： "<<"y="<<k<<"x"<<"+"<<b<<endl;
        cout<<"scanraw直线     "<<i+1<<"   上两点坐标： "<<p3<<",  "<<p4<<endl;
        cout<<"scanraw直线     "<<i+1<<"   方程： "<<"y="<<k2<<"x"<<"+"<<b2<<endl;
        cout<<"scanupraw直线     "<<i+1<<"   上两点坐标： "<<p5<<",  "<<p6<<endl;
        cout<<"scanupraw直线     "<<i+1<<"   方程： "<<"y="<<k3<<"x"<<"+"<<b3<<endl;
        viewer.addLine(p1, p2, 240-40*i, 0, 0, "line_scandown"+to_string(i), 0);
        viewer.addLine(p3, p4, 240-40*i, 0, 0, "line_scanraw"+to_string(i), 0);
        viewer.addLine(p5, p6, 240-40*i, 0, 0, "line_scanupraw"+to_string(i), 0);
    }
    
   viewer.updatePointCloud(scandown_pc,color_down,"pc");
   viewer.updatePointCloud(scanraw_pc,color_raw,"pc2");
   viewer.updatePointCloud(scanupraw_pc,color_upraw,"pc3");
   viewer.spinOnce();
   viewer.removeAllShapes();
#endif

  vector<pair<size_t,TLine2D_My> > res1 = vec4f2Tline(scandown_lines_coeff);
  vector<pair<size_t,TLine2D_My> > res2 = vec4f2Tline(scanraw_lines_coeff);
  vector<pair<size_t,TLine2D_My> > res3 = vec4f2Tline(scanupraw_lines_coeff);
  //cout<<"第"<<f<<"detected_line个大小："<<res1.size()<<endl;
  //cout<<"第"<<f<<"detected_line系数："<<res1[0].second.coefs[0]<<endl;
  detected_lines.clear();
  detected_lines.resize(3);
  detected_lines.push_back(res1);
  detected_lines.push_back(res2);
  detected_lines.push_back(res3);
  //如果前面没有定义detected_lines的大小，那这里就不会有数据，因为没有给detected_lines分配内存。万分注意！！！
  //cout<<"第"<<f<<"之前detected_lines个大小："<<detected_lines[0].size()<<endl;
  //将直线方程转成优化器需要的数据
  generate_cos(M_num_LRFs,detected_lines,vCOs);
  
  //cout<<"第"<<f<<"次回调中，detected_lines大小："<<detected_lines[0].size()<<endl;
  cout<<"第"<<f<<"次回调中，vCOs大小："<<vCOs.size()<<endl;
  
 
  //vcos中数据大于400时候，就跳出这个过程，开始执行spinonce后面的内容
  if(vCOs.size() > 400)
  {
    flag = false;
  }
  //回调函数每执行一次，f加1,刚开始执行的时候，vcos还没有数据，这里可能会报错
  // cout<<"第"<<f<<"次回调中，vCOs的第"<<f<<"个Co里面的第一个Co_1里的第一条直线的方向向量"
  //        <<vCOs[f][0].lines_co[0].dir<<endl; 
  f+=1;
    
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"listener_lidar");
  ros::NodeHandle nh;
  //初始化3个订阅对象，接收3个laserscan消息
  //第一个参数是节点句柄，第二个是话题名称，第三个是字节数，第四个是设置传输协议，一般用无延迟的nodelay
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_raw (nh,"scan_raw",10,ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_down (nh,"scan_down",10,ros::TransportHints().tcpNoDelay());
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_up_raw (nh,"scan_up_raw",10,ros::TransportHints().tcpNoDelay());
  //初始化一个近似的同步时间戳，里面同步的是两个laserscan
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                                                          sensor_msgs::LaserScan,sensor_msgs::LaserScan> syncpolicy;
  //讲上面的时间戳传给同步器，同时传递两个对应的订阅对象
  message_filters::Synchronizer<syncpolicy> sync (syncpolicy(10),scan_raw,scan_down,scan_up_raw);
  //注册回调函数
  //bind是将参数和函数绑定起来，_1,_2默认将上面的两个参数按照顺序传进来，即_1=scan_raw中的laserscan，_2同理
  sync.registerCallback(boost::bind(&process,_1,_2,_3));
  //一直回调，直到vcos数据量大于100，就停止
  while(flag && ros::ok())
  {
      ros::spinOnce();
  }

}