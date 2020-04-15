#ifndef XM_ARM_ROBOT_HARDWARE_H
#define XM_ARM_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <xm_arm_msgs/xm_ArmSerialDatagram.h>

#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>

class ArmRobotHardware : public hardware_interface::RobotHW
{
public:
    ArmRobotHardware(ros::NodeHandle nh);
    ~ArmRobotHardware();
    ros::Time getTime();
    ros::Duration getPeriod();
    ros::CallbackQueue* getCallbackQueue();
    void read(const ros::Time, const ros::Duration period);
    void write(const ros::Time, const ros::Duration period);
    bool start();
    void stop();
    double getFreq() const;
private:
    void publishArmCommand(const u_int8_t func, const u_int8_t jnt_id,
                           const float jnt_pos);
    void publishArmJState(const u_int8_t func, const u_int8_t jnt_id);
    void getArmStateCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
    void getArmStatusCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
    bool checkArmStatus();
    void transPositionJointToActuator();
    void transPositionActuatorToJoint();
private:
    ros::NodeHandle    nh_;
    ros::CallbackQueue callback_queue_;
    ros::Publisher     arm_serial_pub_;
    ros::Subscriber    arm_state_sub_;
    ros::Subscriber    arm_status_sub_;
    int                arm_command_id_;
    int                arm_state_id_;
    double             freq_;//这个频率是什么意思?
    hardware_interface::JointStateInterface    jnt_state_interfece_;
    hardware_interface::PositionJointInterface jnt_position_interface_;
    hardware_interface::VelocityJointInterface jnt_velocity_interface_;
    std::vector<std::string>      jnt_name_;
    std::vector<std::string>      act_name_;
    std::map<std::string, double> jnt_pos_;
    std::map<std::string, double> jnt_vel_;
    std::map<std::string, double> jnt_eff_;
    std::map<std::string, double> act_pos_;
    std::map<std::string, double> jnt_cmd_;
    std::map<std::string, double> act_cmd_;
    std::vector<ros::Time>        jnt_stamp_;
    enum HARDWARE_STATUS {UNKNOWN, READY, RUNNING, ERROR};
    std::vector<HARDWARE_STATUS> jnt_status_;
};

#endif 
