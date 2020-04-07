#include <Client/Client.h>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"moveit_action_client");
    ros::NodeHandle n;
    Client demo(n);

    while(demo.getTrajectoryState().isDone() && ros::ok())
        {
            usleep(50000);
        }
    return 0;
}