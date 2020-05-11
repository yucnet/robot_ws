#ifndef XM_ARM_HARDWARE_SERIAL_NODE_H_
#define XM_ARM_HARDWARE_SERIAL_NODE_H_
//SerialParams 将串口通信使用到的一系列参数封装到这个类中
#include <ros/ros.h>
#include "xm_arm_hardware_serial_node/xm_arm_hardware_serial_port.h"//对Serial_param的定义
#include <xm_arm_msgs/xm_ArmSerialDatagram.h>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

namespace xm_serial_node {
class SerialNode
{
public:
    SerialNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
    virtual ~SerialNode();
private:
    void loadParams();//
   void getDatagramCallback(
       const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr &msg);
    void getSerialCallback(xm_arm_msgs::xm_ArmSerialDatagramPtr ptr_datagram);
private:
    //串口号:
    //波特率:
    //停止位:
    //流控制:防止数据丢失
    //校验位:
    SerialParams                  serial_params_;//自定义类,用于串口通信的参数
    int                           timeout_;
    boost::shared_ptr<SerialPort>        ptr_serial_port_;//抽象串口指针
    ros::NodeHandle               nh_;//
    ros::NodeHandle               private_nh_;//
    ros::Subscriber               serial_sub_;//订阅发布Datagram的话题
    map<int, string>              topic_name_;
    map<u_int8_t, ros::Publisher> serial_pub_;
};

} // namespace xm_serial_node

#endif // XM_ARM_HARDWARE_SERIAL_NODE_H_
