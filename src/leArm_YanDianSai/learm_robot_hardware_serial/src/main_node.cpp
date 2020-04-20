#include <xm_arm_hardware_serial_node/xm_arm_hardware_serial_node.h>
#include <xm_arm_hardware_serial_node/xm_arm_hardware_serial_port.h>
#include<ros/ros.h>
using namespace std;
using namespace xm_serial_node;//该空间内含有了Serial类

int main(int argc, char **argv)
{
    //这是一个硬件串口节点
    ros::init(argc, argv, "xm_arm_hardware_serial_node");
    SerialNode serial_node(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}
