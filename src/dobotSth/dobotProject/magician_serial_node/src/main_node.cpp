#include <dobot_serial_node/dobot_serial_node.h>
#include <ros/ros.h>
using namespace std;
//using namespace learm_serial_node;//该空间内含有了Serial类

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Dobot_robot_node");
    //SerialNode serialNode(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}
