#include <dobotTask/dobotTask.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

vector<int> hsvred    ={0, 0,    10,  50, 255, 55, 255};
vector<int> hsvyellow ={0, 26,    34,  70, 255, 75, 255};
vector<int> hsvblue   ={0, 100, 124,  50, 255, 55, 255};
vector<int> hsvgreen  ={0, 45,   77,  50, 255, 55, 255};
vector<int> hsvpurple ={0, 125, 155,  50, 255, 55, 255};

pthread_mutex_t mutex;

using namespace std;

class Handeye final : public dobotTask
{
public:
    Handeye(ros::NodeHandle& node):
    m_node(node), dobotTask(node)
    {   
        pthread_mutex_init(&mutex, NULL);
        

        this->camera_matrix = (cv::Mat_<double>(3,3) <<   921.386962890625,             0.0, 629.8939819335938,  
                                                                       0.0, 918.67041015625, 361.6900634765625, 
                                                                       0.0,             0.0,               1.0);
        this->dist_coeffs =(cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        this->camera_matix <<  921.386962890625,             0.0, 629.8939819335938,  
                                            0.0, 918.67041015625, 361.6900634765625, 
                                            0.0,             0.0,               1.0;
        this->camera_matix_inverse = camera_matix.inverse();
        
        image_transport::ImageTransport it_(m_node);

        m_point_stamped_publisher = this->m_node.advertise<geometry_msgs::PointStamped>("marker_corners",100);

        m_image_sub = it_.subscribe("/camera/color/image_raw",100,&Handeye::callbackImage,this);
        m_depth_image_sub = it_.subscribe("/camera/aligned_depth_to_color/image_raw",100,&Handeye::alignDepthcallbackImage,this);

        //geometry_msgs::PointStamped point;
        this->camera_stamped_points_msgs.header.frame_id = "camera_color_optical_frame";
    }

    void process(vector <int> hsv_, string color);
    void cameraAxisCalculation(string color);
    void compute(string color);

    void callbackImage(const sensor_msgs::ImageConstPtr& msg);
    void alignDepthcallbackImage(const sensor_msgs::ImageConstPtr& msg);
    void loadCalibrationFiles(string& input_path, cv::Mat& camera_matrix, cv::Mat& distcoeffs, double scale);
    void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center);
    void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center);
    void sendMarkerTf(vector<cv::Vec3d>& marker_vecs, vector<cv::Vec3d>& marker_rotate_vecs);
    void publishTarget2BaseTF();
    void publishWorld2BaseTF();
    void searchTF();

    void workStateJudge();

    void testCalculation();

public:
   
    ros::NodeHandle& m_node;

    tf::TransformBroadcaster camera_to_marker_tf_broadcaster;
    tf::TransformBroadcaster marker_to_base_tf_broadcaster;
    tf::TransformBroadcaster world_to_base_tf_broadcaster;
    tf::TransformListener    listener;
    
    vector< cv::Point2f > marker_center;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    //用于图像检测
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    vector< vector< cv::Point2f > > marker_corners;

    //vector< cv::Point2f >  pixel_points;


    Eigen::Matrix<double,3,3> camera_matix;
    Eigen::Matrix<double,3,3> camera_matix_inverse;
  
    ros::Publisher m_point_stamped_publisher;
    ros::ServiceClient client_pose;
    image_transport::Subscriber m_image_sub;
    image_transport::Subscriber m_depth_image_sub;

    cv::Mat picture;
    cv::Mat depth_align_picture;
    
    vector< vector< cv::Point > > contours;
    vector< cv::Vec4i > hierarcy;
    vector< cv::Point2f > points;//像素坐标
    vector< cv::Point3d > camera_points;
    vector< cv::Point3d > base_points;//机器人坐标系
    //vector< vector <cv::Point2f > > marker_corners

    geometry_msgs::PointStamped camera_stamped_points_msgs;
    geometry_msgs::PointStamped pout;

    map< string, vector< cv::Point2f > > pixel_points;

    string camera_frame = "camera_color_optical_frame";

};


void Handeye::process( vector <int> hsv_, string color )
{
    this->pixel_points[color].clear();

    cv::Mat drawmap = this->picture; 
    cv::Mat clone = picture.clone(); 

    cv::cvtColor(clone,clone,CV_BGR2HSV);
    cv::inRange(clone,cv::Scalar( hsv_[1], hsv_[3], hsv_[5] ),
                    cv::Scalar( hsv_[2], hsv_[4], hsv_[6]),clone );
                
    cv::Mat binary = clone.clone();
    cv::medianBlur(binary,binary,25);

    cv::findContours(clone, this->contours, this->hierarcy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if( this->contours.size() == 0 ) {

        return;

    }

    vector<cv::Rect> rect;
    for(int i = 0;i <this->contours.size(); i++){

        cv::RotatedRect Rrec = minAreaRect(contours[i]); 
        cv::Point2f P[4];
 
		Rrec.points(P);

        rect.push_back( cv::Rect(P[0], P[2]) );
        //rect.push_back(cv::boundingRect(this->contours[i]));

        if ( rect[i].area() > 1500 ) {

            cv::rectangle(drawmap, rect[i],cv::Scalar(0,0,255),3);
            cv::Point2f center(0.5*(rect[i].tl().x+rect[i].br().x), 0.5*(rect[i].tl().y+rect[i].br().y));
            this->points.push_back(center);

            this->pixel_points[color].push_back(center);

        }

    //cout<< "we detected " << this->pixel_points[color].size() << "contour(s)" <<endl;
    }

    cv::imwrite("/home/zhangeaky/color.jpg", drawmap);

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
    cout<<endl<<"回调1: "<<endl;

   

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

    getMarker(cv_ptr->image,this->marker_center);

    if ( isworking == 1 ) {

        ROS_INFO("主线程正在工作");

    }

    this->picture = cv_ptr->image.clone();    
}

void Handeye::alignDepthcallbackImage(const sensor_msgs::ImageConstPtr& msg)
{

    if ( isworking == 1 ) {

        ROS_INFO("主线程正在工作");

    }

    cout<<endl<<" align "<<endl;
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

    this->depth_align_picture = cv_ptr->image;
    cv::medianBlur(this->depth_align_picture, this->depth_align_picture, 3);
    //cameraAxisCalculation("red");
    testCalculation();

}

void Handeye::getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center)
{
    vector<int> ids;
    vector<cv::Vec3d> rotate_vecs, trans_vecs;

    this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    if( !marker_image.empty() ) {
            
        cv::aruco::detectMarkers(marker_image, dictionary, this->marker_corners, ids);
        cv::aruco::drawDetectedMarkers(marker_image, this->marker_corners, ids);
        cv::aruco::estimatePoseSingleMarkers(this->marker_corners, 0.0880, this->camera_matrix, this->dist_coeffs, rotate_vecs, trans_vecs);//0.1645

        if ( rotate_vecs.empty()&&trans_vecs.empty()  ) {

            ROS_ERROR("No Marker detected!!!");
            return;

        } else {
            //cout << "二维码数量:" << ids.size() << endl;
            //cout << "ID of the marker is: " << ids[0] << endl;
            //cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs ,rotate_vecs, trans_vecs, 0.1); 
            //getMarkerCoordinate(corners, ids, marker_center);
        }  

        cv::Mat pic = marker_image.clone();

        for ( int i = 0; i < this->marker_corners[0].size(); i++ ) {

            cv::circle( pic, marker_corners[0][i], 2, (255,0,0), 5 );

        }
        
        cv::imshow("maker", pic);
        cv::waitKey(1);
            //cout<<trans_vecs[0]<<endl;
            
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
    if( marker_rotate_vecs.size()==0&&marker_rotate_vecs.size()==0 ){

            cout<<"haven't received any vecs yet"<<endl;

    } else {

            cv::Mat rotated_matrix(3, 3, CV_64FC1);

        
            cv::Rodrigues(marker_rotate_vecs[0],rotated_matrix);
            rotated_matrix.convertTo(rotated_matrix, CV_64FC1);

            tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<double>(0,0), rotated_matrix.at<double>(0,1),rotated_matrix.at<double>(0,2),
                                rotated_matrix.at<double>(1,0), rotated_matrix.at<double>(1,1), rotated_matrix.at<double>(1,2),
                                rotated_matrix.at<double>(2,0), rotated_matrix.at<double>(2,1), rotated_matrix.at<double>(2,2));

            tf::Vector3 tf_tvec(marker_trans_vecs[0][0],marker_trans_vecs[0][1],marker_trans_vecs[0][2]);
          
            tf::Transform transform(tf_rotated_matrix, tf_tvec);
            this->camera_to_marker_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "target_marker"));  
            this->publishTarget2BaseTF();
            //ROS_INFO("TF successfully sent!"); 
    } 
}

void Handeye::publishTarget2BaseTF()
{ 
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin( tf::Vector3( -0.0205, 0.2635, 0.0000) );

    q.setRPY(0.00,0.00,-1.57);
    transform.setRotation(q);

    this->marker_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"target_marker","magician_base"));

    transform.setOrigin( tf::Vector3( 0.0000, 0.0000, 0.127424) );

    q.setRPY(0.00,0.00,0.00);
    transform.setRotation(q);

    this->marker_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"magician_base","magician_origin"));
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

void Handeye::cameraAxisCalculation( string color )
{

    if ( this->marker_corners.size() == 0 ) {

        ROS_ERROR(" NO marker_corners returned!");

        return;

    } else {
        
        // if ( pixel_points[color].size() == 0 ) {

        //     ROS_ERROR("No %s detected", color);

        //     return;

        // }

        //遍历每种颜色的每一个像素点得到深度值,并得到机器人坐标系的坐标
        for ( int i = 0; i < this->marker_corners[0].size(); i++ ) {

            cout<< "角点: "<<marker_corners[0].size() <<endl;

            float Zc;
            Zc = this->depth_align_picture.at< unsigned short >(marker_corners[0][i].y,marker_corners[0][i].x );
            cout<<"第"<<i<<"个角点的深度值是"<<Zc<<endl;
            Zc /= 1000;

            double Xcam = ( marker_corners[0][i].x - this->camera_matrix.at<double>(0,2) )*Zc*( 1/this->camera_matrix.at<double>(0,0) );
            double Ycam = ( marker_corners[0][i].y - this->camera_matrix.at<double>(1,2) )*Zc*( 1/this->camera_matrix.at<double>(1,1) );
            float  Zcam = Zc;

            ros::Time now = ros::Time::now();

            geometry_msgs::PointStamped pin;

            pin.header.frame_id = "camera_color_optical_frame";
            
            //pin.header.frame_id = "camera_color_frame";

            pin.header.stamp = now;

            pin.point.x = Xcam;
            pin.point.x = Ycam;
            pin.point.x = Zcam;

            cout<< Xcam <<" : "<< Ycam <<" : "<< Zcam <<endl;

            this->m_point_stamped_publisher.publish( pin );

            geometry_msgs::PointStamped pout;
           
            try {

                // this->listener.waitForTransform("camera_color_optical_frame", "target_marker",
                //               ros::Time(0), ros::Duration(3.0));
                this->listener.transformPoint("target_marker", ros::Time(0),  pin, "camera_color_optical_frame", pout);

            } catch ( tf::TransformException &ex ) {

                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();

            }   
            //this->m_point_stamped_publisher.publish( pout );

            targetpoints[color].push_back( cv::Point3f( pout.point.x, pout.point.y,pout.point.z ) ); 

        }
    }
}



void Handeye::testCalculation( )
{
        for ( int i = 0; i < this->marker_corners[0].size(); i++ ) {

            cout<< "二维码角点: "<<marker_corners[0].size() <<endl;

            float Zc;
            Zc = this->depth_align_picture.at< unsigned short >(marker_corners[0][i].y,marker_corners[0][i].x );
            cout<<"第"<<i<<"个角点的深度值是"<<Zc<<endl;
            Zc /= 1000;

            double Xcam = ( marker_corners[0][i].x - this->camera_matrix.at<double>(0,2) )*Zc*( 1/this->camera_matrix.at<double>(0,0) );
            double Ycam = ( marker_corners[0][i].y - this->camera_matrix.at<double>(1,2) )*Zc*( 1/this->camera_matrix.at<double>(1,1) );
            float  Zcam = Zc;


            camera_stamped_points_msgs.point.x = Xcam;
            camera_stamped_points_msgs.point.y = Ycam;
            camera_stamped_points_msgs.point.z = Zc;
            camera_stamped_points_msgs.header.stamp = ros::Time::now();
            //this->m_point_stamped_publisher.publish(camera_stamped_points_msgs);

            geometry_msgs::PointStamped pout;
           
            try {

                // this->listener.waitForTransform("camera_color_optical_frame", "target_marker",
                //               ros::Time(0), ros::Duration(3.0));
                this->listener.transformPoint("target_marker", ros::Time(0),  camera_stamped_points_msgs, "camera_color_optical_frame", pout);

            } catch ( tf::TransformException &ex ) {

                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();

            } 

            this->m_point_stamped_publisher.publish(pout);  
            cout<< "x: " <<pout.point.x << "y: " << pout.point.y << "z: " << pout.point.z <<endl;

        }
}
