#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dobot/SetVoiceTaskCmd.h>
#include <dobot/SetCmdTimeout.h>
#include <dobot/SetJOGCmd.h>
#include <cstdlib>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
class VoiceTaskServer
{
public:
    VoiceTaskServer(ros::NodeHandle& node):
    m_node(node)
    {
        this->m_cmdtimeout_client = m_node.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
        this->m_timeout_srv.request.timeout = 3000;
        if (this->m_cmdtimeout_client.call(this->m_timeout_srv) == false) {
            ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        }

        this->m_jog_client = m_node.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
        this->m_voice_server = this->m_node.advertiseService("voice/dobot",&VoiceTaskServer::SetVoiceTask,this);
    }
    bool SetVoiceTask(dobot::SetVoiceTaskCmd::Request &req, dobot::SetVoiceTaskCmd::Response &res)
    {
        this->m_jog_srv.request.isJoint=1;
        
        switch(req.voiceOrder1) {
            case 1:
                ROS_INFO("W");
                this->m_jog_srv.request.cmd = 1;
                this->m_jog_client.call(this->m_jog_srv);
                break;
            case 2:
                ROS_INFO("S");
                this->m_jog_srv.request.cmd = 2;
                this->m_jog_client.call(this->m_jog_srv);
            break;
            case 3:
                ROS_INFO("A");
                this->m_jog_srv.request.cmd = 3;
                this->m_jog_client.call(this->m_jog_srv);
            break;
            // case KEYCODE_D:
            //     ROS_INFO("D");
            //     srv.request.cmd = 4;
            // break;
            // case KEYCODE_U:
            //     ROS_INFO("U");
            //     srv.request.cmd = 5;
            // break;
            // case KEYCODE_I:
            //     ROS_INFO("I");
            //     srv.request.cmd = 6;
            // break;
            // case KEYCODE_J:
            //     ROS_INFO("J");
            //     srv.request.cmd = 7;
            // break;
            // case KEYCODE_K:
            //     ROS_INFO("K");
            //     srv.request.cmd = 8;
            // break;
            default:
                //ROS_INFO("DEFAULT:0x%02x", );
                //srv.request.cmd = 0;
            break;
      
           }  return true;
    }

    void JogTest()
    {
        this->m_jog_srv.request.isJoint = 0;
        this->m_jog_srv.request.cmd = 1;
        int c = 5;
        // while(c){

            this->m_jog_client.call(this->m_jog_srv);
            c--;
        // }
        // c = 5;
        // this->m_jog_srv.request.cmd = 2;
        //  while(c){

        //     this->m_jog_client.call(this->m_jog_srv);
        //     c--;
        // }
        
    }
public:
    ros::NodeHandle m_node;
    ros::ServiceServer m_voice_server;


    ros::ServiceClient m_cmdtimeout_client;
    ros::ServiceClient m_jog_client;
    
    dobot::SetCmdTimeout m_timeout_srv;
    dobot::SetJOGCmd m_jog_srv;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient_JOG");
    ros::NodeHandle n;
    VoiceTaskServer x(n);
    x.JogTest();

    ros::spin();
   
}

