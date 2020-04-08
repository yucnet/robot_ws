#include <client/client.h>
using namespace std;
int main(int argc, char** argv)
{
    ROS_INFO("Begin!");
    ros::init(argc,argv,"moveit_action_client");
    ros::NodeHandle n;
    moveitClient demo(n);

    while(demo.getTrajectoryState().isDone() && ros::ok())
        {
            cout<<"进入主循环"<<endl;
            usleep(5000);
        }
    cout<<"debug!"<<endl;
    ros::spin();
    return 0;
}


