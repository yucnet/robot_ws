#include <ros/ros.h>
#include <learm_hardware_serial_node/learm_hardware_serial_port.h>//对Serial_param的定义
#include <learm_robot_msgs/GoalPoint.h>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

namespace learm_serial_node {
class SerialNode
{
public:
    SerialNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
    virtual ~SerialNode();
private:
    void loadParams();//加载串口参数
    void getCommandDataCallback(
       const learm_robot_msgs::GoalPoint::ConstPtr& msg);
    void getSerialCallback(learm_robot_msgs::GoalPointConstPtr ptr_datagram);

private:
    //串口号:
    //波特率:
    //停止位:
    //流控制:防止数据丢失
    //校验位:
    SerialParams                  serial_params_;//自定义类,用于串口通信的参数
    int                           timeout_;//?????????????????????????????
    boost::shared_ptr<SerialPort>        ptr_serial_port_;//抽象串口指针
    ros::NodeHandle               nh_;//公有node 可以用来进行订阅和发布
    ros::NodeHandle               private_nh_;//私有node可以进行数据获取
    ros::Subscriber               serial_sub_;//订阅发布Datagram的话题
    map<int, string>              topic_name_;
    map<u_int8_t, ros::Publisher> serial_pub_;
};

} // namespace learm_serial_node

