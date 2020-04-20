#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <learm_robot_msgs/GoalPoint.h>

#include <thread>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
using namespace std;
class ArmRobotHardware: public hardware_interface::RobotHW
{
public:
    ArmRobotHardware(ros::NodeHandle nh);
    ~ArmRobotHardware();
    inline ros::Time getTime()
        {
            return ros::Time::now(); 
        }
    inline ros::Duration getPeriod()
        {
            return ros::Duration(1 / freq_);
        }
    inline ros::CallbackQueue* getCallbackQueue()
        {
            return &callback_queue_;
        }
    //void read(const ros::Time, const ros::Duration period);
    void write(const ros::Time, const ros::Duration period);
    bool start();
    inline void stop()
        {
            nh_.shutdown();
        }
    inline double getFreq() const
        {
            return this->freq_;
        }
public:
    void publishArmCommand();
    //void publishArmJState(const u_int8_t func, const u_int8_t jnt_id);
    /*void getArmStateCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
    void getArmStatusCallback(
        const xm_arm_msgs::xm_ArmSerialDatagram::ConstPtr& msg);
        */
    void getUpValueCallback(const learm_robot_msgs::GoalPoint::ConstPtr& msg);

    bool checkArmStatus();
    void transPositionJointToActuator();
    void transPositionActuatorToJoint();
    
public:
    ros::NodeHandle    nh_;
    ros::CallbackQueue callback_queue_;
    ros::Publisher     arm_serial_pub_;
    ros::Subscriber    joint_data_sub;

    //ros::Subscriber    arm_state_sub_;
    //ros::Subscriber    arm_status_sub_;
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
    std::vector<ros::Time>        jnt_stamp_;//时间戳,用来计算速度
    enum HARDWARE_STATUS {UNKNOWN, READY, RUNNING, ERROR};
    std::vector<HARDWARE_STATUS> jnt_status_;
};
