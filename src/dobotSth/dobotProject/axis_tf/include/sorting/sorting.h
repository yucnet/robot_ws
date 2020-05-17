#ifndef SORTING_H
#define SORTING_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>

using namespace std;

class sorting
{
public:
    sorting(ros::NodeHandle& node_);

    void callback(const sensor_msgs::ImageConstPtr& msg);
    void process( vector< int >* ptr );


public:
    vector<int> hsv_red    ={0, 0,10, 50,255, 55,255};
    // vector<int> hsvyellow ={0, 26,34,   70,255, 75,255};
    // vector<int> hsvblue   ={0, 100,124, 50,255, 55,255};
    // vector<int> hsvgreen  ={0, 45,77,   50,255, 55,255};
    // vector<int> hsvpurple ={0, 125,155, 50,255, 55,255};
    vector < cv::Point2f > points;
    ros::NodeHandle node;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    cv::Mat picture;
    vector< vector< cv::Point > >  contours;
    vector< cv::Vec4i > hierarcy;
};

#endif