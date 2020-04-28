#include <learm_hardware_serial_node/learm_hardware_serial_node.h>
using namespace std;
namespace learm_serial_node {

SerialNode::SerialNode(const ros::NodeHandle &nh,
                       const ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh)
{
    cout << "SerialNode Object created!" << endl;
    // topic_name_[3] = "status";
    // topic_name_[4] = "state";

    loadParams();//串口通信参数设置
    ptr_serial_port_ = boost::make_shared<SerialPort>();//创建Serialports类,指针形式
    ptr_serial_port_->setSerialParams(serial_params_);//串口参数初始化
    ptr_serial_port_->setTimeOut(timeout_);//延时时间赋值??????????????????????????????????????????????????????
    //ptr_serial_port_->setCallbackFunc(bind(&SerialNode::getSerialCallback, this, _1));
    serial_sub_ = nh_.subscribe("learm_serial/send_arm_command",1000,&SerialNode::getCommandDataCallback,this);
    ptr_serial_port_->startThread();
}


SerialNode::~SerialNode()
{
    ptr_serial_port_->stopThread();
}

void SerialNode::loadParams()
{
    //给参数类赋值
    serial_params_.serial_port_  = "/dev/ttyUSB0";//串口号
    serial_params_.baud_rate_    = 9600;
    serial_params_.flow_control_ = 0;
    serial_params_.parity_bits_  = 0;
    serial_params_.stop_bits_    = 0;
    timeout_                     = 100;

    //????????????????????????????/?????????
    private_nh_.getParam("serial_port_", serial_params_.serial_port_);
    private_nh_.getParam("baud_rate_", (int&)(serial_params_.baud_rate_));
    private_nh_.getParam("flow_control_", (int&)(serial_params_.flow_control_));
    private_nh_.getParam("parity_bits_", (int&)(serial_params_.parity_bits_));
    private_nh_.getParam("stop_bits_", (int&)(serial_params_.stop_bits_));
    private_nh_.getParam("timeout", timeout_);
}

void SerialNode::getCommandDataCallback(
    const learm_robot_msgs::GoalPoint::ConstPtr &msg)
{
    cout << "Sending new datagram!" << endl;
    ptr_serial_port_->writeDataGram(*msg);
}

/*
void SerialNode::getSerialCallback(
    learm_robot_msgs::GoalPointPtr ptr_datagram)
{
    ros::Publisher &pub = serial_pub_[ptr_datagram->receiver];

    if (!pub)
    {
        stringstream ss;
        ss << "xm_arm_serial/receive_arm_" <<
            topic_name_[(int)(ptr_datagram->receiver)];
        pub = nh_.advertise<xm_arm_msgs::xm_ArmSerialDatagram>(ss.str(), 1000);
    }

    pub.publish(ptr_datagram);
}
*/
} 