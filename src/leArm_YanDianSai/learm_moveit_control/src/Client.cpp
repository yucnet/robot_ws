#include <Client/Client.h>

Client::Client(ros::NodeHandle& node):
    node(node),
    moveit_client(node,"learm_controller/follow_joint_trajectory"),
    group("test_1")
{
    //trajectory_msgs::trajectory
    this->joints_name = {"joint1","joint2"," joint3"," joint4","joint5"};
    
    group.setPoseReferenceFrame("base_link");//设定姿态的参考坐标
    group.allowReplanning(true);//允许重新规划
    
    group.setGoalPositionTolerance(0.01);//单位米
    group.setGoalOrientationTolerance(0.05);//单位弧度

    //给五个关节命名
    goal.trajectory.joint_names = this->joints_name;
    goal.trajectory.points.resize(2);
    //
    goal.trajectory.points[trajectory_index_].positions.resize(5);
    goal.trajectory.points[trajectory_index_].positions[0] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[1] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[2] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[3] = 0.0;
    goal.trajectory.points[trajectory_index_].positions[4] = 0.0;
   
    goal.trajectory.points[trajectory_index_].velocities.resize(5);
    goal.trajectory.points[trajectory_index_].accelerations.resize(5);

    for (size_t i = 0; i < 5; ++i)
        {
            goal.trajectory.points[trajectory_index_].velocities[i] = 0.0;
            goal.trajectory.points[trajectory_index_].accelerations[i] = 0.0;
        }

    goal.trajectory.points[trajectory_index_].time_from_start = ros::Duration(5.0);
    ROS_INFO("First trajectory prepare ok!");

    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    this->moveit_client.sendGoal(goal); 
    ROS_INFO_STREAM("Send joint trajectory goal to server successfully!");

    // goal.trajectory.header.frame_id = "base_link";
    // goal.trajectory.header.stamp = ros::Time::now();
    //group.setNamedTarget("Home");
}

actionlib::SimpleClientGoalState Client::getTrajectoryState()
{
    return moveit_client.getState();
}