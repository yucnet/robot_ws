#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"

#include "opencv2/core.hpp"
#include "opencvtest/pointQueue.h"
#include "iostream"
#include "vector"
#include "ctime"


#include <iostream>
using namespace std;
Eigen::Vector3d a(278,417,1);
Eigen::Vector3d b(282,292,1);
Eigen::Vector3d c(406,295,1);
Eigen::Vector3d d(415,423,1);
Eigen::Vector3d e(350,354,1);

vector < Eigen::Vector3d > test_pixel_points = {a,b,c,d,e};
vector < Eigen::Vector3d > test_camera_points;
vector < Eigen::Vector3d > test_world_points;
geometry_msgs::PointStamped test_camera;
geometry_msgs::PointStamped test_world;

class cubePoints
{
public:
    cubePoints(string name):name(name)
    {}
public:
    string name;
    int numbers;
    double distance;
    vector< Eigen::Vector3d > world_points;
    vector< Eigen::Vector3d > camera_points;
    vector< Eigen::Vector3i > pixel_points; 

};

class dobotMagician
{
public:
ros::NodeHandle n;
ros::ServiceClient client;
ros::Subscriber sub_pointcloud;
ros::Publisher publisher;
ros::Subscriber sub_camera_points;

int counter = 0;

geometry_msgs::PointStamped camera;
geometry_msgs::PointStamped world;
geometry_msgs::PointStamped base;
tf::TransformListener listener;
tf::StampedTransform transform_storage;
tf::StampedTransform transform_storage_;
tf::StampedTransform transform_color_depth_frame;

cubePoints green;
cubePoints red;

public:
dobotMagician():green("green"),red("red")
{   
    this->sub_camera_points = n.subscribe("marker_corners_in_the_camera",1000,&dobotMagician::callback,this); 
    this->camera.header.stamp=ros::Time();
    this->camera.header.frame_id="camera_color_optical_frame";
    this->world.header.stamp = ros::Time();
    this->world.header.frame_id = "world";
    this->publisher = n.advertise<geometry_msgs::PointStamped>("marker_corners_in_the_world",1000,this);
    
/****************************************DOBOT_INITIALIZE****************************************************/
    this->client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
    }
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);
    if(srv2.response.result == 0)
    {
        ROS_INFO("Device Queued command clear!");
    }
    else
    {
        ROS_ERROR("Error!");
    }

    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);
    if(srv3.response.result == 0)
    {
        ROS_INFO("Device SetQueuedCmdSatrtExec command clear!");
    }
    else
    {
        ROS_ERROR("Error!");
    }
    

    client = n.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    do {
    client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
    dobot::SetPTPJointParams srv6;

    for (int i = 0; i < 4; i++) {
        srv6.request.velocity.push_back(100);
    }
    for (int i = 0; i < 4; i++) {
        srv6.request.acceleration.push_back(100);
    }
    client.call(srv6);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv7;

        srv7.request.xyzVelocity = 100;
        srv7.request.xyzAcceleration = 100;
        srv7.request.rVelocity = 100;
        srv7.request.rAcceleration = 100;
        client.call(srv7);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv8;

        srv8.request.jumpHeight = 20;
        srv8.request.zLimit = 200;
        client.call(srv8);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv9;

        srv9.request.velocityRatio = 50;
        srv9.request.accelerationRatio = 50;
        client.call(srv9);
    } while (0);
    ROS_INFO("Dobot Initialize has been finished!");


    // client = n.serviceClient<opencvtest::pointQueue>("pixel_Axis");
    // opencvtest::pointQueue  srv10;
    // client.call(srv10);
    // green.numbers = srv10.response.green_num;
    // red.numbers = srv10.response.red_num;
    // cout<<red.numbers<<endl;
    // cout<<green.numbers<<endl;
   
}

void inverseCameraMatrix(double zc);
void dotask();
void pickup(cubePoints& points);
void pixelToCameraToWorld(cubePoints& cubepoints);
void pixelToCameraToWorldTest(vector< Eigen::Vector3d >&);
void callback(const geometry_msgs::PointStampedConstPtr& msgs);

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient_");
    dobotMagician dm;
    //dm.pixelToCameraToWorldTest(test_pixel_points);
    ros::spin();
    return 0;
}

// void dobotMagician::pixelToCameraToWorld(cubePoints& cubepoints)
// {
//     this->client = n.serviceClient<opencvtest::pointQueue>("pixel_Axis");
//     opencvtest::pointQueue srv;
//     client.call(srv);

//     if(srv.response.green_num!=0)
//         {
//             for(int i = 0;i<srv.response.green_num;i++)
//                 {
//                     cout<<endl<<"*******************"<<endl;
//                     cout<<"处理绿色:"<<endl;
//                     //临时的三维double向量将srv中的值读出
//                     Eigen::Vector3d temp = {srv.response.green[3*i],srv.response.green[3*i+1],1};
//                     cout<<"像素坐标: "<<"("<<temp[0]<<","<<temp[1]<<","<<temp[3]<<")"<<endl;
//                     this->Zc = srv.response.green[3*i+2];
//                     cout<<"尺度因子: "<<this->Zc<<endl; 
//                     this->inverseCameraMatrix(this->Zc);
//                     green.camera_points.push_back(inverse_camera_matrix*temp);
                    

//                     cout<<"camera: "<<endl;
//                     this->camera.point.x = green.camera_points[i][0];
//                     this->camera.point.y = green.camera_points[i][1];
//                     this->camera.point.z = srv.response.green[3*i+2];

//                     try
//                         {
//                             this->listener.waitForTransform("world","camera_color_optical_frame",ros::Time(0), ros::Duration(3.0));
//                             this->listener.lookupTransform("world","camera_color_optical_frame", ros::Time(0), this->transform_storage);         
//                         }
//                     catch (tf::TransformException &ex) 
//                         {
//                             ROS_ERROR("%s",ex.what());
//                             ros::Duration(1.0).sleep();
//                         }

//                     try
//                         {
//                             this->listener.transformPoint("world",camera, world);
//                             green.world_points[i][0] = world.point.x;
//                             green.world_points[i][1] = world.point.y;
//                             green.world_points[i][2] = world.point.z;
//                         }
//                     catch (tf::TransformException &ex) 
//                         {
//                             ROS_ERROR("%s",ex.what());
//                             ros::Duration(1.0).sleep();
//                         }
//                     cout<<"world:"<<"("<<world.point.x<<","<<world.point.y<<","<<world.point.z<<")"<<endl; 
//                     cout<<"*******************"<<endl;
//                 }
//         }
//     /*********************************************************************************************************************/    
//      if(srv.response.red_num!=0)
//         {
//               for(int i = 0;i<srv.response.green_num;i++)
//                 {
//                     cout<<endl<<"*******************"<<endl;
//                     cout<<"处理绿色:"<<endl;
//                     //临时的三维double向量将srv中的值读出
//                     Eigen::Vector3d temp = {srv.response.green[3*i],srv.response.green[3*i+1],1};
//                     cout<<"像素坐标: "<<"("<<temp[0]<<","<<temp[1]<<","<<temp[3]<<")"<<endl;
//                     this->Zc = srv.response.green[3*i+2];
//                     cout<<"尺度因子: "<<this->Zc<<endl; 
//                     this->inverseCameraMatrix(this->Zc);
//                     green.camera_points.push_back(inverse_camera_matrix*temp);
                    

//                     cout<<"camera: "<<endl;
//                     this->camera.point.x = green.camera_points[i][0];
//                     this->camera.point.y = green.camera_points[i][1];
//                     this->camera.point.z = srv.response.green[3*i+2];

//                     try
//                     {
//                         this->listener.waitForTransform("world","camera_color_optical_frame",ros::Time(0), ros::Duration(3.0));
//                         this->listener.lookupTransform("world","camera_color_optical_frame", ros::Time(0), this->transform_storage);
//                         //this->listener.lookupTransform("camera_color_optical_frame","camera_depth_optical_frame",
//                         //ros::Time(0),this->transform_color_depth_frame);(402,318)

//                         //this->listener.lookupTransform("dobot_base","world", ros::Time(0),this->transform_storage_);           
//                     }
//                     catch (tf::TransformException &ex) 
//                         {
//                             ROS_ERROR("%s",ex.what());
//                             ros::Duration(1.0).sleep();
//                         }
//                     try
//                         {
//                             this->listener.transformPoint("world",camera, world);
//                             //ROS_INFO("Get the world points of green.");
//                             green.world_points[i][0] = world.point.x;
//                             green.world_points[i][1] = world.point.y;
//                             green.world_points[i][2] = world.point.z;
//                             //this->listener.transformPoint("dobot_base",world, base);
//                         }
//                     catch (tf::TransformException &ex) 
//                         {
//                             ROS_ERROR("%s",ex.what());
//                             ros::Duration(1.0).sleep();
//                         }
//                     cout<<"world:"<<"("<<world.point.x<<","<<world.point.y<<","<<world.point.z<<")"<<endl; 
//                     cout<<"*******************"<<endl;
//                 }
//         }
// }

// void dobotMagician::pixelToCameraToWorldTest(vector< Eigen::Vector3d >& pixel_axis)
// {
//     cout<<"working!"<<endl;
//     test_camera.header.stamp = ros::Time();
//     test_camera.header.frame_id = "camera_color_optical_frame";

//     test_world.header.stamp = ros::Time();
//     test_world.header.frame_id = "world";
   

//     ros::Rate r(10);

//     while(ros::ok())
//     {
//         for(int i = 0;i<pixel_axis.size();i++)
//             {
//                 cout<<endl<<endl;
//                 cout<<"**********************************************************"<<endl;
//                 cout<<"像素坐标: "<<"("<<pixel_axis[i][0]<<","<<pixel_axis[i][1]<<","<<pixel_axis[i][2]<<")";
//                 cout<<"----->像相机坐标: ";
//                 Eigen::Vector3d temp = {pixel_axis[i][0],pixel_axis[i][1],pixel_axis[i][2]};
//                 this->inverseCameraMatrix(zc[i]);
//                 test_camera_points.push_back(inverse_camera_matrix*temp);
//                 test_camera.point.x = test_camera_points[i][0];
//                 test_camera.point.y = test_camera_points[i][1];
//                 test_camera.point.z = zc[i];
//                 cout<<"("<<test_camera.point.x<<","<<test_camera.point.y<<","<<test_camera.point.z<<")"<<"----->世界坐标: ";
//                 this->publisher.publish(test_camera);
            

//                 try
//                     {
//                         this->listener.waitForTransform("world","camera_color_optical_frame",ros::Time(0), ros::Duration(3.0));
//                         //this->listener.lookupTransform("world","camera_color_optical_frame", ros::Time(0), this->transform_storage);
//                         this->listener.lookupTransform("world","camera_color_optical_frame",ros::Time(0),this->transform_storage);     
//                     }
//                 catch (tf::TransformException &ex) 
//                     {
//                         ROS_ERROR("%s",ex.what());
//                         ros::Duration(1.0).sleep();
//                     }
//                 try
//                     {
//                         this->listener.transformPoint("world",test_camera, test_world);
//                         cout<<"("<<test_world.point.x*1000<<","<<test_world.point.y*1000<<","<<
//                         test_world.point.z*1000<<")"<<endl;
                        
//                         this->client = this->n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
//                         // dobot::SetPTPCmd srv;
//                         // srv.request.x = test_world.point.x*1000;
//                         // srv.request.y = test_world.point.y*1000;
//                         // srv.request.z = test_world.point.z*1000;
//                         // client.call(srv);
//                         // sleep(5000);
//                     }
//                 catch (tf::TransformException &ex) 
//                     {
//                         ROS_ERROR("%s",ex.what());
//                         ros::Duration(1.0).sleep();
//                     }
//             }
            
//             cout<<"**********************************************************"<<endl;
//             r.sleep();
//     }
// }

void dobotMagician::pickup(cubePoints& cubepoints)
{
    this->client = this->n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv;

    for(int i;i < cubepoints.world_points.size();i++)
    {
        srv.request.x = cubepoints.world_points[i][0];
        srv.request.y = cubepoints.world_points[i][1];
        srv.request.z = cubepoints.world_points[i][2];
        this->client.call(srv);
    }
}

void dobotMagician::callback(const geometry_msgs::PointStampedConstPtr& msgs)
{
    counter++;
    geometry_msgs::PointStamped camera,world;
    camera.header.frame_id=msgs->header.frame_id;
    camera.header.stamp = ros::Time();
    world.header.stamp = ros::Time();
    world.header.frame_id = "world";
    //如果不加异常处理,会导致程序因出错而强行终止,出现核心已转储等模糊的错误.
    try{
        this->listener.waitForTransform("world","camera_color_optical_frame",ros::Time(0), ros::Duration(3.0));
        this->listener.lookupTransform("world","camera_color_optical_frame", ros::Time(0), this->transform_storage);         
        this->listener.transformPoint("world",camera, world);
        publisher.publish(world);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    

    // cout<<"("<<world.point.x*1000<<","<<world.point.y*1000<<","<<world.point.z*1000<<") ";
    // if(counter%4==0)
    //     cout<<endl;

}