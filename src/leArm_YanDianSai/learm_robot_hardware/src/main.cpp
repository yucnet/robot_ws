#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <learm_robot_hardware/learm_robot_hardware.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "learm_robot_hardware");
    ros::NodeHandle node;
    ArmRobotHardware learm_robothw(node);

    //初始化工作
    if (learm_robothw.start())
        ROS_INFO_STREAM("learm_robot_hardware initialize successfully!");
    else
    {
        ROS_ERROR_STREAM("learm_robot_hardware initialize by error!");
        return 0;
    }

    ros::NodeHandle cm_nh("learm_controller_manager");
    ros::CallbackQueue cm_callback_queue;
    cm_nh.setCallbackQueue(&cm_callback_queue);
    //传入硬件实例和一个硬件管理节点
    controller_manager::ControllerManager manager(&learm_robothw, cm_nh);
    ros::Rate rate(learm_robothw.getFreq());
    ros::AsyncSpinner hw_spinner(1, learm_robothw.getCallbackQueue());
    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
    hw_spinner.start();
    cm_spinner.start();

    while (ros::ok())
    {
        ROS_INFO_STREAM("进入主循环"<<" "<<endl);
        ros::Time current_time = ros::Time::now();
        learm_robothw.read(current_time,
            ros::Duration(1 / learm_robothw.getFreq()));
        manager.update(current_time,
            ros::Duration(1 / learm_robothw.getFreq()));
        learm_robothw.write(current_time,
            ros::Duration(1 / learm_robothw.getFreq()));
        rate.sleep();
    }

    hw_spinner.stop();
    cm_spinner.stop();
    return 0;
}
