#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <dobot/SetCmdTimeout.h>
#include <dobot/SetQueuedCmdClear.h>
#include <dobot/SetQueuedCmdStartExec.h>
#include <dobot/SetQueuedCmdForceStopExec.h>
#include <dobot/GetDeviceVersion.h>
#include <dobot/SetEndEffectorParams.h>
#include <dobot/SetPTPJointParams.h>
#include <dobot/SetPTPCoordinateParams.h>
#include <dobot/SetPTPJumpParams.h>
#include <dobot/SetPTPCommonParams.h>
#include <dobot/SetPTPCmd.h>

using namespace std;

class Dobot
{
public:
    Dobot(ros::NodeHandle& node):
        m_node(node)
    {
        m_subscriber = m_node.subscribe("magician_robot/send_arm_data",1000,&Dobot::callback,this);
        m_client = m_node.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
        m_waypoint_counter = 0;

        //SetCmdTimeout
        dobot::SetCmdTimeout srv1;
        srv1.request.timeout = 3000;
        if (m_client.call(srv1) == false) {
            ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
            return;
        }

        // Clear the command queue
        m_client = m_node.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
        dobot::SetQueuedCmdClear srv2;
        m_client.call(srv2);

        // Start running the command queue
        m_client = m_node.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
        dobot::SetQueuedCmdStartExec srv3;
        m_client.call(srv3);
        
        // Get device version information
        m_client = m_node.serviceClient<dobot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
        dobot::GetDeviceVersion srv4;
        m_client.call(srv4);
        if (srv4.response.result == 0) {
            ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
        } else {
            ROS_ERROR("Failed to get device version information!");
        }

        // Set end effector parameters
        m_client = m_node.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
        dobot::SetEndEffectorParams srv5;
        srv5.request.xBias = 70;
        srv5.request.yBias = 0;
        srv5.request.zBias = 0;
        m_client.call(srv5);

        // Set PTP joint parameters
        do {
            m_client = m_node.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
            dobot::SetPTPJointParams srv;

            for (int i = 0; i < 4; i++) {
                srv.request.velocity.push_back(100);
            }
            for (int i = 0; i < 4; i++) {
                srv.request.acceleration.push_back(100);
            }
            m_client.call(srv);
        } while (0);

        // Set PTP coordinate parameters
        do {
            m_client = m_node.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
            dobot::SetPTPCoordinateParams srv;
            srv.request.xyzVelocity = 100;
            srv.request.xyzAcceleration = 100;
            srv.request.rVelocity = 100;
            srv.request.rAcceleration = 100;
            m_client.call(srv);
        } while (0);

        // Set PTP jump parameters
        do {
            m_client = m_node.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
            dobot::SetPTPJumpParams srv;

            srv.request.jumpHeight = 20;
            srv.request.zLimit = 200;
            m_client.call(srv);
        } while (0);

        // Set PTP common parameters
        do {
            m_client = m_node.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
            dobot::SetPTPCommonParams srv;

            srv.request.velocityRatio = 50;
            srv.request.accelerationRatio = 50;
            m_client.call(srv);
        } while (0);
    }

    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        cout<<"goal: "<<this->m_waypoint_counter<<endl;
        m_client = m_node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd srv;
        srv.request.ptpMode = 4;
        srv.request.x = msg->data[0];
        srv.request.y = msg->data[1];
        srv.request.z = msg->data[2];
        this->m_client.call(srv);
    }

public:
    ros::NodeHandle m_node;
    ros::ServiceClient m_client;
    ros::Subscriber m_subscriber;
    int m_waypoint_counter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle node;
    Dobot dobot(node);
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    ros::spin();
} 

       


