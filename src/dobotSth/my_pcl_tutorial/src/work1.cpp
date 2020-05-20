#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

class pcltutorial
{
  public:
  pcltutorial( ros::NodeHandle& node ): node(node) {
    cout<<"begin"<<endl;
    this->pub = node.advertise< sensor_msgs::PointCloud2 >( "/camera/depth/color/points/processed", 1 );
    this->sub = this->node.subscribe("/camera/depth/color/points", 1,&pcltutorial::cloud_cb,this);
  }

  void cloud_cb ( const sensor_msgs::PointCloud2ConstPtr& cloud_msg )
  {
    pcl::PointXYZRGB
  #if 0
    ROS_INFO( "点云处理!" );
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    //将ros格式的点云数据转化成 pcl库的点云数据
    pcl_conversions::toPCL( *cloud_msg, *cloud );

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.5f, 0.5f, 0.5f);
    sor.filter (cloud_filtered);

    sensor_msgs::PointCloud2 output;

    //将pcl格式的点云数据转化为ros格式数据发布
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    // Publish the data
    pub.publish (output);
  #endif

    pcl::PCLPointCloud2Ptr cloud ( new pcl::PCLPointCloud2() );
    reader.read( "/home/zhangeaky/table_scene_lms400.pcd", *cloud );

    pcl_conversions::moveFromPCL( *cloud, output );

    pub.publish(output);
  }

  void pubpointcloud()
  {
    pcl::PCLPointCloud2Ptr cloud ( new pcl::PCLPointCloud2);
    reader.read( "/home/zhangeaky/table_scene_lms400.pcd", *cloud );

    pcl_conversions::moveFromPCL( *cloud, output );

    pub.publish(output);

  }

  public:

  ros::NodeHandle node;
  ros::Publisher pub;
  ros::Subscriber sub; 
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  

  sensor_msgs::PointCloud2 output;

  pcl::PCLPointCloud<pcl::PointXYZRGBA>




};

int main ( int argc, char** argv )
{
  ros::init( argc, argv, "my_pcl_tutorial" );
  ros::NodeHandle nh;
  pcltutorial zhangeaky(nh);
  ros::spin ();
}