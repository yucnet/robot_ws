/**********************ROS****************************************/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
/*********************EUGEN***************************************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
/********************OPENCVLIBRARY********************************/
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
/*******************PACKAGE_HEADER*******************************/
#include <dobot/GetPose.h>
#include <opencvtest/pointQueue.h>
/*********************VARIBE***********************************/
#include<unistd.h>
using namespace std;

class Transform
{
private:
ros::NodeHandle n;
ros::Publisher camera_axis_pub;
ros::ServiceClient client_pose;

image_transport::ImageTransport it;
image_transport::Subscriber image_sub_;
image_transport::Subscriber aligned_image_sub;
ros::Publisher publisher;

tf::TransformBroadcaster marker_position_broadcaster;
tf::TransformBroadcaster magician_origin_bra;
tf::TransformBroadcaster dobot_effector_bra;

cv::Matx33d camera_matrix_for_estimate;
Eigen::Matrix3d camera_matrix_for_cal;
Eigen::Matrix3d inverse_camera_matrix;
cv::Mat dist_coeffs; 

vector < vector<cv::Point2f> > corners;
cv::Ptr<cv::aruco::Dictionary> dictionary;
vector<int> ids;
vector<cv::Vec3d> rvecs, tvecs;
vector<cv::Point2f> marker_center;

vector< vector<double> > object_points;
vector< vector<int> > image_points;
vector<double> tvector;
vector<double> rvector;

cv::Mat global_paper;

double Zc;

public:
Transform():it(n)
{   
    cout<<getpid()<<endl;
    this->client_pose = n.serviceClient<dobot::GetPose>("dobot/GetPose");
    this->publisher = this->n.advertise<geometry_msgs::PointStamped>("marker_corners_in_the_camera",1000,this);
    
    this->image_sub_ = it.subscribe("/camera/color/image_raw",1,&Transform::callbackImage,this);
    this->aligned_image_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw",
    1,&Transform::callbackAlignedImage,this);
    this->camera_matrix_for_estimate << 615.1227416992188,          0.000000,   327.1946716308594,
                                                 0.000000, 614.7720336914062,  242.58395385742188, 
                                                 0.000000,          0.000000,            1.000000;

    this->camera_matrix_for_cal << 615.1227416992188,          0.000000,  327.1946716308594,
                                            0.000000, 614.7720336914062,  242.58395385742188, 
                                            0.000000,          0.000000,  1.000000;

    this->dist_coeffs = (cv::Mat_<double>(1,5) << 0.000, 0.000, 0.000, 0.000, 0.000);
    this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);   
}

//回调函数
void callbackImage(const sensor_msgs::ImageConstPtr& msg);
void callbackAlignedImage(const sensor_msgs::ImageConstPtr& msg);

void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center, bool key);
void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center);
void sendMarkerTf(vector<cv::Vec3d>& marker_vecs, vector<cv::Vec3d>& marker_rvecs);
void sendDobotTf();

void inverseCameraMatrix(double zc);
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "axis_tf");
    Transform p;
    ros::spin();
}

void Transform::callbackAlignedImage(const sensor_msgs::ImageConstPtr& msg)
    {
        ROS_INFO("ALIGNED_IMAGE");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat paper = cv_ptr->image.clone();
        if(this->corners.size() == 0)
            {
                cout<<"no corner"<<endl;
                return;
            }else{
                for(int i=0;i< this->corners[0].size();i++)
                    {
                      
                        Zc = cv_ptr->image.at<unsigned short>(corners[0][i].x,corners[0][i].y)/1000.00;
                        cv::putText(this->global_paper,std::to_string(Zc),corners[0][i],1,1.5,cv::Scalar(0,0,255),1,8);
                        if(Zc == 0)
                            {
                                cout<<"XXXXX"<<endl;
                            }else{
                            cout<<Zc<<endl;
                        }
                        
                        geometry_msgs::PointStamped point;
                        point.header.frame_id = "camera_color_optical_frame";
                        point.header.stamp = ros::Time();
                        Eigen::Vector3d temp = {corners[0][i].x,corners[0][i].y,1};
                        this->inverseCameraMatrix(Zc);
                        Eigen::Vector3d camera_axis_point= inverse_camera_matrix*temp;
                        point.point.x = camera_axis_point[0];
                        point.point.y = camera_axis_point[1];
                        point.point.z = Zc;
                        this->publisher.publish(point);
                    }
            }
        
       
        cv::imshow("AlignedImage",paper);
        cv::waitKey(1);
    }

void Transform::callbackImage(const sensor_msgs::ImageConstPtr& msg)
    {   
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        this->global_paper = cv_ptr->image.clone();
        this->getMarker(global_paper,marker_center,0);
        cv::imshow("callbackImage",global_paper);
        cv::waitKey(1);
    }


void Transform::getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center, bool key)
    {
        if(!marker_image.empty())
            {
                cv::aruco::detectMarkers(marker_image, dictionary, corners, ids);
                if(ids.size()>0 )
                {
                    // cout<<"-----------------------------"<<endl;
                    // cout<<"角点:"<<endl;
                    for(int i = 0;i<corners[0].size();i++)
                        {
                            cv::circle(marker_image,corners[0][i],2,cv::Scalar(255,0,0),5);
                        }
                      cv::aruco::drawDetectedMarkers(marker_image, corners, ids);

                    this->getMarkerCoordinate(corners,ids,marker_center);
                    cv::aruco::estimatePoseSingleMarkers(corners,0.076, this->camera_matrix_for_estimate, dist_coeffs, rvecs, tvecs);
                    
                    if(rvecs.empty()&&tvecs.empty())
                        {
                            cout<<"no trans"<<endl;
                        }
                    else
                        {
                            cv::aruco::drawAxis(marker_image, this->camera_matrix_for_estimate, dist_coeffs ,rvecs, tvecs, 0.1); 
                        }  
                    sendMarkerTf(rvecs,tvecs);
                    }
                    else
                    {
                        ROS_INFO("No Marker detected.");
                    }
                    
              
            }
    }
void Transform::getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center)
    {
        cv::Point2f center(0.f, 0.f);
        for(int i = 0;i < corners[0].size();i++)
            {
                center += corners[0][i];
            }
        center /= 4.0;
        
       // cout<<marker_center[0].x<<","<<marker_center[1].y<<endl;
    }

void Transform::sendMarkerTf(vector<cv::Vec3d>& marker_rvecs,vector<cv::Vec3d>& marker_tvecs)                 
    {
        if(marker_rvecs.size()==0&&marker_rvecs.size()==0)
            {
                cout<<"haven't received any vecs yet"<<endl;
            }
            else
                {

                    cv::Mat rotated_matrix(3, 3, CV_64FC1);//储存旋转矩阵
                    cv::Rodrigues(marker_rvecs[0],rotated_matrix);//旋转向量转换为旋转矩阵
                    rotated_matrix.convertTo(rotated_matrix, CV_64FC1);
                    tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<double>(0,0), rotated_matrix.at<double>(0,1),rotated_matrix.at<double>(0,2),
                                    rotated_matrix.at<double>(1,0), rotated_matrix.at<double>(1,1), rotated_matrix.at<double>(1,2),
                                        rotated_matrix.at<double>(2,0), rotated_matrix.at<double>(2,1), rotated_matrix.at<double>(2,2));

                    tf::Vector3 tf_tvec(marker_tvecs[0][0],marker_tvecs[0][1],marker_tvecs[0][2]);
                    tf::Transform transform(tf_rotated_matrix, tf_tvec);
                    this->marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_color_optical_frame", "world"));  
                    this->sendDobotTf();
                }
    }
    
void Transform::sendDobotTf()
    { 
        cv::Mat rotated_matrix(3,3,CV_64FC1);
        rotated_matrix.at<float>(0,0)= 1.0;rotated_matrix.at<float>(0,1)= 0.0;rotated_matrix.at<float>(0,2)= 0.0;
        rotated_matrix.at<float>(1,0)= 0.0;rotated_matrix.at<float>(1,1)= 1.0;rotated_matrix.at<float>(1,2)= 0.0;
        rotated_matrix.at<float>(2,0)= 0.0;rotated_matrix.at<float>(2,1)= 0.0;rotated_matrix.at<float>(2,2)= 1.0;

        tf::Matrix3x3 tf_rotated_matrix(
        rotated_matrix.at<float>(0,0), rotated_matrix.at<float>(0,1), rotated_matrix.at<float>(0,2),
        rotated_matrix.at<float>(1,0), rotated_matrix.at<float>(1,1), rotated_matrix.at<float>(1,2),
        rotated_matrix.at<float>(2,0), rotated_matrix.at<float>(2,1), rotated_matrix.at<float>(2,2));

        tf::Vector3 tf_tvecs(0.080,0.280, 0.00);
        tf::Transform transform(tf_rotated_matrix, tf_tvecs);
        
        this->magician_origin_bra.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","magician_origin"));
        //this->sendDobotEffectorTF();
    }

void Transform::inverseCameraMatrix(double zc)
{
    this->inverse_camera_matrix = this->camera_matrix_for_cal.inverse(); 
    this->inverse_camera_matrix(0,0) *=  zc;
    this->inverse_camera_matrix(0,2) *=  zc;
    this->inverse_camera_matrix(1,1) *=  zc;
    this->inverse_camera_matrix(1,2) *=  zc;
    this->inverse_camera_matrix(2,2) *=  zc;
}
