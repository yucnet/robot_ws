#include <learm_robot_hardware/learm_robot_hardware.h>
ArmRobotHardware::ArmRobotHardware(ros::NodeHandle nh)
    : nh_(nh),
      freq_(20)
{
    nh_.setCallbackQueue(&callback_queue_);

//设置关键和电机的名称
    jnt_name_.push_back("joint1");
    jnt_name_.push_back("joint2");
    jnt_name_.push_back("joint3");
    jnt_name_.push_back("joint4");
    jnt_name_.push_back("joint5");
    jnt_name_.push_back("joint6");
    jnt_name_.push_back("joint7");

    act_name_.push_back("id_1");
    act_name_.push_back("id_2");
    act_name_.push_back("id_3");
    act_name_.push_back("id_4");
    act_name_.push_back("id_5");
    act_name_.push_back("id_6");
    act_name_.push_back("id_7");

    //初始化,每一个关节的转角,速度,力矩等都设为0
    for (int i = 0; i < jnt_name_.size(); i++)
        {
            jnt_pos_[jnt_name_[i]] = 0.0;
            jnt_vel_[jnt_name_[i]] = 0.0;
            jnt_eff_[jnt_name_[i]] = 0.0;
            jnt_cmd_[jnt_name_[i]] = 0.0;
            act_cmd_[act_name_[i]] = 0.0;
            act_pos_[act_name_[i]] = 0.0;

            hardware_interface::JointStateHandle jnt_state_handle(jnt_name_[i],
                &jnt_pos_[jnt_name_[i]], &jnt_vel_[jnt_name_[i]],
                &jnt_eff_[jnt_name_[i]]);
            jnt_state_interfece_.registerHandle(jnt_state_handle);
            hardware_interface::JointHandle jnt_position_handle(
                jnt_state_interfece_.getHandle(jnt_name_[i]),
                &jnt_cmd_[jnt_name_[i]]);
            jnt_position_interface_.registerHandle(jnt_position_handle);
        }

    registerInterface(&jnt_state_interfece_);
    registerInterface(&jnt_position_interface_);

    for (int i = 0; i < 7; i++)
        {
            jnt_stamp_.push_back(ros::Time::now());
            jnt_status_.push_back(UNKNOWN);
        }

    arm_command_id_ = 3;//??
    arm_state_id_   = 4;

    arm_serial_pub_ = nh_.advertise<learm_robot_msgs::GoalPoint>(
        "learm_serial/send_arm_command", 1000);//发送给穿串口节点,让其发送给串口下位机
    /*
    arm_state_sub_ = nh_.subscribe("xm_arm_serial/receive_arm_state", 1000,
        &ArmRobotHardware::getArmStateCallback, this);
    arm_status_sub_ = nh_.subscribe("xm_arm_serial/receive_arm_status", 1000,
        &ArmRobotHardware::getArmStatusCallback, this);
    */
}

ArmRobotHardware::~ArmRobotHardware()
{
    nh_.shutdown();
}

//机械臂关节角度 到 电机角度的映射
void ArmRobotHardware::transPositionJointToActuator()
{
    //关节与相应的控制电机要一一对应
    act_cmd_["id_1"]     = jnt_cmd_["joint1"];
    act_cmd_["id_2"]     = jnt_cmd_["joint2"];
    act_cmd_["id_3"]     = jnt_cmd_["joint3"];
    act_cmd_["id_4"]     = jnt_cmd_["joint4"];
    act_cmd_["id_5"]     = jnt_cmd_["joint5"];
    act_cmd_["id_6"]     = jnt_cmd_["joint6"];
    act_cmd_["id_7"]     = jnt_cmd_["joint7"];
}

// Because autuator's direction is not unified, the signs of positive and
// negative may have problem in the following formula.
void ArmRobotHardware::transPositionActuatorToJoint()
{
    jnt_cmd_["joint1"] =  act_cmd_["id_1"] ;
    jnt_cmd_["joint2"] =  act_cmd_["id_2"] ;
    jnt_cmd_["joint3"] =  act_cmd_["id_3"] ;
    jnt_cmd_["joint4"] =  act_cmd_["id_4"] ;
    jnt_cmd_["joint5"] =  act_cmd_["id_5"] ;
    jnt_cmd_["joint6"] =  act_cmd_["id_6"] ;
    jnt_cmd_["joint7"] =  act_cmd_["id_7"] ;
}


/* 接受反馈
//接受底层 消息
void ArmRobotHardware::getArmStateCallback(
    const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg)
{
    const u_int8_t *data_ptr = msg->data.data();
    size_t jnt_index = msg->sender - 0x2A;

    if (*data_ptr != 0x01)
        return ;
    if (*(data_ptr + 1) != 0x00)
    {
        ROS_ERROR_STREAM("Reading joint's states from embedded system failed!");
        jnt_status_[jnt_index] = ERROR;
        return ;
    }

    float data = *(float *)(data_ptr + 2);
    ros::Time current_time = ros::Time::now();
    float delta_time = (current_time - jnt_stamp_[jnt_index]).toSec();
    jnt_stamp_[jnt_index] = current_time;

    act_pos_[jnt_name_[jnt_index]] = data;
    jnt_vel_[jnt_name_[jnt_index]] = 0;
    jnt_eff_[jnt_name_[jnt_index]] = 0;

    if (jnt_status_[jnt_index] == UNKNOWN)
    {
        jnt_status_[jnt_index] = READY;
        return ;
    }
    else
        jnt_status_[jnt_index] = RUNNING;
}

void ArmRobotHardware::getArmStatusCallback(
    const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr &msg)
{
    const u_int8_t *data_ptr = msg->data.data();
    size_t jnt_index = msg->sender - 0x2A;

    if (*data_ptr == 0x01)
    {
        if(*(data_ptr + 1) != 0x00)
        {
            ROS_ERROR_STREAM(
                "Writing joint's commands to embedded system failed!");
            jnt_status_[jnt_index] = ERROR;
            return ;
        }
        else
            jnt_status_[jnt_index] = RUNNING;
    }
}
*/

/*
发布 关节状态
void ArmRobotHardware::publishArmJState(const u_int8_t func,
                                        const u_int8_t jnt_id)
{
    xm_arm_msgs::xm_ArmSerialDatagramPtr datagram_ptr =
        boost::make_shared<xm_arm_msgs::xm_ArmSerialDatagram>();
    datagram_ptr->sender = arm_state_id_;
    datagram_ptr->receiver = jnt_id + 0x2A;
    datagram_ptr->data.resize(1, 0);
    u_int8_t *data_ptr = datagram_ptr->data.data();
    data_ptr[0] = func;
    arm_serial_pub_.publish(datagram_ptr);
}
*/


void ArmRobotHardware::publishArmCommand(const u_int8_t func,
                                         const u_int8_t jnt_id,
                                         const float    jnt_pos)
{
    learm_robot_msgs::GoalPointPtr data_ptr =
        boost::make_shared<learm_robot_msgs::GoalPoint>();
    // datagram_ptr->sender = arm_command_id_;
    // datagram_ptr->receiver = jnt_id + 0x2A;
    // datagram_ptr->data.resize(5, 0);
    // u_int8_t *data_ptr = datagram_ptr->data.data();
    // data_ptr[0] = func;
    // *(float *)(data_ptr + 1) = jnt_pos;
    // arm_serial_pub_.publish(datagram_ptr);
    data_ptr->joint1 = 1500;
    data_ptr->joint2 = 1500;
    data_ptr->joint3 = 1500;
    data_ptr->joint4 = 1500;
    data_ptr->joint5 = 1500;
    data_ptr->joint6 = 1500;
    data_ptr->joint7 = 1500;
    arm_serial_pub_.publish(data_ptr);
}

bool ArmRobotHardware::checkArmStatus()
{
    for (int i = 0; i < 7; i++)
        {
            //joint_status的值
            if (jnt_status_[i] == ERROR)
                {
                    ROS_ERROR_STREAM("%s state error" << jnt_name_[i]);
                    return false;
                }
        }

    return true;
}

/* 读取
void ArmRobotHardware::read(const ros::Time, const ros::Duration period)
{

    for (size_t i = 0; i < 6; i++)
        publishArmJState(0x01, i);

    transPositionActuatorToJoint();

    callback_queue_.callAvailable(ros::WallDuration(1 / freq_ / 3));
    ros::Time current_time = ros::Time::now();

    for (size_t i = 0; i < 6; i++)
    {
        if ((current_time - jnt_stamp_[i]).toSec() > 0.5)
        {
            ROS_WARN_STREAM("Reading timeout!");
            return ;
        }
    }
}
*/
void ArmRobotHardware::write(const ros::Time, const ros::Duration period)
{
    transPositionJointToActuator();

    for (size_t i = 0; i < 7; i++)
        publishArmCommand(0x01, i, act_cmd_[act_name_[i]]);

    ROS_INFO_STREAM("joint_command");
    ROS_INFO_STREAM(
        "joint1: " << jnt_cmd_["joint1"] <<endl<<
        "joint2: " << jnt_cmd_["joint2"] <<endl<<
        "joint3: " << jnt_cmd_["joint3"] <<endl<<
        "joint4: " << jnt_cmd_["joint4"] <<endl<<
        "joint5: " << jnt_cmd_["joint5"] <<endl<<
        "joint6: " << jnt_cmd_["joint6"] <<endl<<
        "joint7: " << jnt_cmd_["joint7"] <<endl);
    ROS_INFO_STREAM("------");

    callback_queue_.callAvailable(ros::WallDuration(1 / freq_ / 3));
    ros::Time current_time = ros::Time::now();

    for (size_t i = 0; i < 7; i++)
    {
        if ((current_time - jnt_stamp_[i]).toSec() > 0.5)
        {
            ROS_WARN_STREAM("Writing timeout!");
            return ;
        }
    }
}

bool ArmRobotHardware::start()
{
    for (size_t i = 0; i < 7; i++)
        {
            //publishArmJState(0x01, i);
            publishArmCommand(0x01, i, 0.0);
        }

    ROS_INFO_STREAM("Starting to read and write joint's states!");
    callback_queue_.callAvailable(ros::WallDuration(1 / freq_));

    if (!checkArmStatus())
        {
            ROS_ERROR_STREAM("Initialize Failed!");
            return false;
        }else
            return true;
}