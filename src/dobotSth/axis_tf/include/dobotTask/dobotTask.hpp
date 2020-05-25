#include "ros/ros.h"

#include <map>

#include <opencv2/core/core.hpp>

#include "dobot/SetPTPCmd.h"
#include "dobot/GetDeviceSN.h"
#include "dobot/GetAlarmsState.h"
#include "dobot/ClearAllAlarmsState.h"
#include "dobot/SetEndEffectorParams.h"
#include <dobot/SetEndEffectorSuctionCup.h>
#include <dobot/GetHOMEParams.h>
#include <dobot/SetHOMECmd.h>
#include <dobot/GetPose.h>

using namespace std;

class dobotTask 
{

    public:

    dobotTask(ros::NodeHandle& node) : node(node){

        this->alarmClear();
        this->endEffectorParamsSet();

    }

    ~dobotTask() = default;


    void getDeviceSN();
    void goHome();



    void alarmState();
    void alarmClear();
       
    void updateCurrentPose();
    void goToPoint();
    void pointLimitJudge();

    void endEffectorParamsSet();
    void pick();
    void place();

    void ifDestination();

    
 

    public:

    ros::NodeHandle node;
    ros::ServiceClient client;
    ros::ServiceClient m_get_pose = node.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    ros::ServiceClient m_home_client = node.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
    ros::ServiceClient m_PTP_client = node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    ros::ServiceClient m_suction_client = node.serviceClient<dobot::SetEndEffectorSuctionCup>
                                                    ("/DobotServer/SetEndEffectorSuctionCup");
    ros::ServiceClient m_endeffector_params;

    vector<int> current_pose;

    map< string, vector< cv::Point3f > > targetpoints;
    //vector< cv::Point3f>  

};

void dobotTask::updateCurrentPose()
{
    client = this->node.serviceClient<dobot::GetPose>("/DobotServer/GetPose");

    dobot::GetPose srv;
    client.call(srv);
    this->current_pose.push_back(static_cast<int>(srv.response.x));
    this->current_pose.push_back(static_cast<int>(srv.response.y));
    this->current_pose.push_back(static_cast<int>(srv.response.z));
}

void dobotTask::goHome()
{
    dobot::SetHOMECmd srv;
    this->m_home_client.call(srv);

    if( srv.response.result == 0 ){
        cout<<"回零完成!"<<endl;
    } else {
        cout<<"回零出错!"<<endl;        
    }
    
}

void dobotTask::goToPoint()
{

    this->client = this->node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd srv;
    srv.request.ptpMode = 0;
    srv.request.x = 0;
    srv.request.y = 0;
    srv.request.z = 10;
    client.call(srv);

}

void dobotTask::endEffectorParamsSet()
{
    this->client = node.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");

    dobot::SetEndEffectorParams srv;
    srv.request.xBias = 0;
    srv.request.yBias = 0;
    srv.request.zBias = 0;

}

void dobotTask::pointLimitJudge()
{
   

}

void dobotTask::getDeviceSN()

{
    dobot::GetDeviceSN srv;
    this->client = node.serviceClient<dobot::GetDeviceSN>("/DobotServer/GetDeviceSN");
    client.call(srv);
    cout<<"设备的序列号: "<<srv.response.deviceSN<<endl;
}

void dobotTask::alarmState()
{
    this->client = node.serviceClient<dobot::GetAlarmsState>("/DobotServer/GetAlarmsState");
    dobot::GetAlarmsState srv;

    cout<<"size: "<<srv.response.alarmsState.size()<<endl;

    for ( int i = 0; i < srv.response.alarmsState.size(); i++ ) {

        cout<<srv.response.alarmsState[i]<<endl;

    }
    
}

void dobotTask::alarmClear()
{
    this->client = this->node.serviceClient<dobot::ClearAllAlarmsState>("/DobotServer/ClearAllAlarmsState");
    dobot::ClearAllAlarmsState srv;
    client.call(srv);
    cout<<"clearAlarm result :"<<srv.response.result<<endl;
}

void dobotTask::pick()
{
    this->client = node.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");

    dobot::SetEndEffectorSuctionCup srv;
    srv.request.enableCtrl = 1;//使能
    srv.request.suck = 1; //吸气
    client.call(srv);
   
    cout<<"吸气结果: "<<srv.response.result<<endl;
}

void dobotTask::place()
{

    this->client = node.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");

    dobot::SetEndEffectorSuctionCup srv;

    srv.request.enableCtrl = 0;
    //srv.request.suck = 1; //吸气

    client.call(srv);

  
    cout<<"吐气结果: "<<srv.response.result<<endl;

}