#include<ros/ros.h>
#include<dobot/GetPose.h>
#include<sensor_msgs/JointState.h>
using namespace std;
class JointPublish
{
    public:
    ros::NodeHandle n;
    dobot::GetPose pose;
    sensor_msgs::JointState jointState;
    ros::Publisher pub; 
    ros::ServiceClient client; 

    public:
    JointPublish(ros::NodeHandle& node):
    n(node)
    {
        this->pub = n.advertise<sensor_msgs::JointState>("joint_states",1000);
       
        this->client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    }
    ~JointPublish()
    {}
    void upDate()
    {
        this->client.call(this->pose);
       
        this->jointState.header.frame_id = "dobot_base";
        this->jointState.header.stamp = ros::Time::now();
        this->jointState.name = {"magician_joint1","magician_joint2","magician_joint3"};
        this->jointState.position = {this->pose.response.jointAngle[0]*(3.1415926/180),
                                     this->pose.response.jointAngle[1]*(3.1415926/180),
                                     this->pose.response.jointAngle[2]*(3.1415926/180)};
        pub.publish(jointState);
        for(int i=0;i<pose.response.jointAngle.size();i++)
            {
                this->jointState.header.stamp = ros::Time::now();
                this->jointState.position;
                cout<<"关节:"<<pose.response.jointAngle[i]*(3.1415926/180)<<endl;
            }
        cout<<pose.response.x<<" "<<pose.response.y<<" "<<pose.response.z<<endl;
    }
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Dobot_joint_states_publisher");
    ros::NodeHandle node;
    
    JointPublish x(node);
    ros::Rate r(10);
    while(ros::ok())
    {
        x.upDate();
        r.sleep();
    }
    ros::spin();
    return 0;
}