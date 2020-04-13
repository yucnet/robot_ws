#include <ros/ros.h>
#include <dobot/GetPose.h>
#include "dobot/SetHOMECmd.h"
using namespace std;
class getPose
{
public: 
    getPose()
    {
        ROS_INFO("Show Pose!");
        this->client = n.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
        this->client.call(this->srv_home);
        this->client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        this->dobot_pose = {0,0,0};
        while(true)
            {
                this->client.call(this->srv);
            
                if(dobot_pose[0]!=srv.response.x&&dobot_pose[1]!=srv.response.y&&dobot_pose[2]!=srv.response.z)
                    {
                        cout<<endl<<"####################"<<endl;
                        cout<<"x: "<<srv.response.x<<endl;
                        cout<<"y: "<<srv.response.y<<endl;
                        cout<<"z: "<<srv.response.z<<endl;
                        cout<<"####################"<<endl;
                    }
                dobot_pose[0]=srv.response.x;
                dobot_pose[1]=srv.response.y;
                dobot_pose[2]=srv.response.z;   
            }
    }
private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    dobot::SetHOMECmd srv_home;
    dobot::GetPose srv;
    vector<double> dobot_pose;
};
int main(int argc, char** argv)
{
    ros::init(argc,argv,"pose");
    getPose p;
}