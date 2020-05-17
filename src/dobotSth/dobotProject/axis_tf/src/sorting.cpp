#include "sorting/sorting.h"

sorting::sorting( ros::NodeHandle& node_):
    node(node_)
{
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&sorting::callback, this);//订阅节点，该节点在USB_cam/image_raw中发布。
    //center_point_pub_= nh_.advertise<opencvtest::pixel_point>("pixel_center_axis",1000);//话题名称和接受队列缓存消息条数；
    //cv::namedWindow();




}

























