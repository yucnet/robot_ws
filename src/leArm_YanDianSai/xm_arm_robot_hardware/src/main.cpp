#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <xm_arm_robot_hardware/xm_arm_robot_hardware.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_arm_robot_hardware");
    ros::NodeHandle xm_nh;
    ArmRobotHardware xm_arm_robothw(xm_nh);

    if (xm_arm_robothw.start())
        ROS_INFO_STREAM("xm_arm_robot_hardware initialize successfully!");
    else
    {
        ROS_ERROR_STREAM("xm_arm_robot_hardware initialize by error!");
        return 0;
    }

    ros::NodeHandle cm_nh("xm_arm");
    ros::CallbackQueue cm_callback_queue;
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager manager(&xm_arm_robothw, cm_nh);
    ros::Rate rate(xm_arm_robothw.getFreq());
    ros::AsyncSpinner hw_spinner(1, xm_arm_robothw.getCallbackQueue());
    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
    hw_spinner.start();
    cm_spinner.start();

    while (ros::ok())
    {
        ros::Time current_time = ros::Time::now();
        xm_arm_robothw.read(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        manager.update(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        xm_arm_robothw.write(current_time,
            ros::Duration(1 / xm_arm_robothw.getFreq()));
        rate.sleep();
    }

    hw_spinner.stop();
    cm_spinner.stop();
    return 0;
}
