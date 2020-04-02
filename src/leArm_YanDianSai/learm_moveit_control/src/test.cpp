#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <actionlib/client/action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
using namespace std;
typedef actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> client;
vector<string> joint_name;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  joint_name.push_back("joint_1");
  joint_name.push_back("joint_2");
  joint_name.push_back("joint_3");
  joint_name.push_back("joint_4");
  joint_name.push_back("joint_5");

  moveit::planning_interface::MoveGroupInterface group("test_1");
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.726282;
  target_pose1.orientation.x= 4.04423e-07;
  target_pose1.orientation.y = -0.687396;
  target_pose1.orientation.z = 4.81813e-07;

  target_pose1.position.x = 0.03;
  target_pose1.position.y = 0.2;
  target_pose1.position.z = 0.5;
  group.setPoseTarget(target_pose1);

  client client("learm_controller/follow_joint_trajectory");
  ROS_INFO("waiting fro server to response!");
  client.waitForActionServerToStart();

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = joint_name;
  //msg.points




  control_msgs::FollowJointTrajectoryGoal goal;

  //goal.trajectory.header
  goal.trajectory.joint_names = joint_name;





  
  return 0;
}
