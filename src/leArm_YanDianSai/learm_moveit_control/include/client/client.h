#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
using namespace std;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client;

class moveitClient{
public:
    int trajectory_index_ = 0;
    client moveit_client;

    control_msgs::FollowJointTrajectoryGoal goal;
    control_msgs::FollowJointTrajectoryResult result;
    control_msgs::FollowJointTrajectoryFeedback feedb;

    actionlib::SimpleClientGoalState getTrajectoryState();

    ros::NodeHandle& node;

    trajectory_msgs::JointTrajectory traj;
    control_msgs::JointTolerance tol;
    moveit::planning_interface::MoveGroupInterface group;
    vector<string> joints_name;


public:
    moveitClient(ros::NodeHandle& node);
    ~moveitClient();
    void activeCb();
    void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void doneCb(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result);
};
