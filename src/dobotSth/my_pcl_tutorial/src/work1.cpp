#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>


int main ( int argc, char** argv )
{
  ros::init( argc, argv, "my_pcl_tutorial" );
  ros::NodeHandle nh;
  cv::waitKey(0);
  //pcltutorial zhangeaky(nh);
  ros::spin ();
  
}