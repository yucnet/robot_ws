#include <learm_hardware_serial_node/learm_hardware_serial_node.h>
#include <learm_hardware_serial_node/learm_hardware_serial_port.h>
#include<ros/ros.h>
using namespace std;
using namespace learm_serial_node;//该空间内含有了Serial类

int main(int argc, char **argv)
{
    ros::init(argc, argv, "learm_serial_node");
    SerialNode serialNode(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}
