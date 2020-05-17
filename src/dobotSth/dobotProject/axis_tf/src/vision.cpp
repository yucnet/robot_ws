/**********************ROS****************************************/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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

using namespace std;
class Handeye
{
public:
    Handeye(ros::NodeHandle& node):
    m_node(node)

    {
        cout<<"debug"<<endl;   
        this->camera_matrix = (cv::Mat_<double>(3,3) <<   921.386962890625,             0.0, 629.8939819335938,  
                                                                       0.0, 918.67041015625, 361.6900634765625, 
                                                                       0.0,             0.0,               1.0);
        this->dist_coeffs =(cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
        image_transport::ImageTransport it_(m_node);
        //client_pose = m_node.serviceClient<dobot::GetPose>("dobot/GetPose");
        m_image_sub = it_.subscribe("/camera/color/image_raw",100,&Handeye::callbackImage,this);
    }

    void cameraAxisCalculation();
    void callbackImage(const sensor_msgs::ImageConstPtr& msg);
    void loadCalibrationFiles(string& input_path, cv::Mat& camera_matrix, cv::Mat& distcoeffs, double scale);
    void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center);
    void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center);
    void sendMarkerTf(vector<cv::Vec3d>& marker_vecs, vector<cv::Vec3d>& marker_rotate_vecs);
    void publishTarget2BaseTF();
    void publishWorld2BaseTF();
    // void sendDobotEffectorTF();
    void searchTF();


public:
    ros::NodeHandle m_node;
    
    vector<cv::Point2f> marker_center;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    vector< vector< cv::Point2f > > marker_corners;
  
    tf::TransformBroadcaster camera_to_marker_tf_broadcaster;
    tf::TransformBroadcaster marker_to_base_tf_broadcaster;
    tf::TransformBroadcaster world_to_base_tf_broadcaster;
    tf::TransformListener    listener;

    ros::ServiceClient client_pose;
    image_transport::Subscriber m_image_sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_eye"); 
    ros::NodeHandle n;

    Handeye x(n);
    
    ros::Rate loop_rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void Handeye::searchTF()
{
    tf::StampedTransform transform;
    try{

        listener.lookupTransform("", "/turtle1",  
                                ros::Time(0), transform);
    }catch ( tf::TransformException ex ){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Handeye::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    cout<<msg->header.frame_id<<endl;
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
    cv::Mat paper = cv_ptr->image.clone();
    getMarker(paper,this->marker_center);
    cv::imshow("callbackImage",paper);
    cv::waitKey(1); 
}

void Handeye::getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center)
    {
        vector<int> ids;
        vector< vector<cv::Point2f> > corners;
        vector<cv::Vec3d> rotate_vecs, trans_vecs;
 
        this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        if(!marker_image.empty()){
               
            cv::aruco::detectMarkers(marker_image, dictionary, corners, ids);
            cv::aruco::drawDetectedMarkers(marker_image, corners, ids);
            cv::aruco::estimatePoseSingleMarkers(corners, 0.088, this->camera_matrix, this->dist_coeffs, rotate_vecs, trans_vecs);
               
            if(rotate_vecs.empty()&&trans_vecs.empty()){

                ROS_ERROR("No Marker detected!!!");
                return;
            }
            else{
                cout<<rotate_vecs.size()<<" "<<trans_vecs.size()<<endl;

                cout<<"ID of the marker is: "<<ids[0]<<endl;
                cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs ,rotate_vecs, trans_vecs, 0.1); 
                //getMarkerCoordinate(corners, ids, marker_center);
            }  
                //cout<<trans_vecs[0]<<endl;
                //cv::circle(marker_image,point1,2,(255,0,0),5);
                //cv::circle(marker_image,point2,2,(255,0,0),5);
                //char num[10];
				//sprintf(num,"logitech",camera_index_);
                sendMarkerTf(rotate_vecs,trans_vecs);
        }else{
            ROS_ERROR("check your camera device!");
            return;
        }
    }

void Handeye::getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center)
    {
        cv::Point2f center(0.f, 0.f);
        for(int i = 0;i < corners[0].size();i++)
            {
                center += corners[0][i];
            }
        center /= 4.0;
        marker_center.push_back(center);
        cout<<marker_center[0].x<<","<<marker_center[0].y<<endl;
    }

void Handeye::sendMarkerTf(vector<cv::Vec3d>& marker_rotate_vecs,vector<cv::Vec3d>& marker_trans_vecs)                 
{
    if(marker_rotate_vecs.size()==0&&marker_rotate_vecs.size()==0){

            cout<<"haven't received any vecs yet"<<endl;

    }else{
            //储存旋转矩阵
            cv::Mat rotated_matrix(3, 3, CV_64FC1);
            //旋转向量转换为旋转矩阵
            cv::Rodrigues(marker_rotate_vecs[0],rotated_matrix);
            rotated_matrix.convertTo(rotated_matrix, CV_64FC1);

            tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<double>(0,0), rotated_matrix.at<double>(0,1),rotated_matrix.at<double>(0,2),
                                rotated_matrix.at<double>(1,0), rotated_matrix.at<double>(1,1), rotated_matrix.at<double>(1,2),
                                rotated_matrix.at<double>(2,0), rotated_matrix.at<double>(2,1), rotated_matrix.at<double>(2,2));

            tf::Vector3 tf_tvec(marker_trans_vecs[0][0],marker_trans_vecs[0][1],marker_trans_vecs[0][2]);
          
            tf::Transform transform(tf_rotated_matrix, tf_tvec);
            //transform = transform.inverse();
            //Eigen::Quaterniond q_eigen;
            //tf::quaternionTFToEigen(transform.getRotation(), q_eigen);
            //temp_rot = q_eigen;
            //tf::vectorTFToEigen(transform.getOrigin(), trans);
            //ostringstream oss;
            //oss << "camera_" << ids[0];
            this->camera_to_marker_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "target_marker"));  
            this->publishTarget2BaseTF();
            ROS_INFO("TF successfully sent!");
            
    } 
}

void Handeye::publishTarget2BaseTF()
{ 
    // cv::Mat rotated_matrix(3,3,CV_64FC1);
    // rotated_matrix.at<float>(0,0)=  0.0; rotated_matrix.at<float>(0,1)= 1.0; rotated_matrix.at<float>(0,2)= 0.0;
    // rotated_matrix.at<float>(1,0)= -1.0; rotated_matrix.at<float>(1,1)= 0.0; rotated_matrix.at<float>(1,2)= 0.0;
    // rotated_matrix.at<float>(2,0)=  0.0; rotated_matrix.at<float>(2,1)= 0.0; rotated_matrix.at<float>(2,2)= 1.0;

    // tf::Matrix3x3 tf_rotated_matrix(
    // rotated_matrix.at<float>(0,0), rotated_matrix.at<float>(0,1), rotated_matrix.at<float>(0,2),
    // rotated_matrix.at<float>(1,0), rotated_matrix.at<float>(1,1), rotated_matrix.at<float>(1,2),
    // rotated_matrix.at<float>(2,0), rotated_matrix.at<float>(2,1), rotated_matrix.at<float>(2,2));
    tf::Transform transform;
    transform.setOrigin( tf::Vector3( -0.0205, 0.2635, 0.0000) );

    tf::Quaternion q;
    q.setRPY(0.00,0.00,0.00);
    transform.setRotation(q);
    this->marker_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"target_marker","magician_base"));
}

void Handeye::publishWorld2BaseTF()
{
    tf::Transform transform_;
    tf::Quaternion q_;
    q_.setRPY(0.00,0.00,0.00);
    transform_.setRotation(q_);
    transform_.setOrigin( tf::Vector3( 0.0000, 0.0000, -0.127424) );
    this->world_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),"world","magician_base"));
}