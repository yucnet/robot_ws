#include <server/server.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"actionlib_moveit_server");
    ros::NodeHandle node;
    moveitServer server(node);
    ros::spin();
}