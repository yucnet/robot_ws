#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <std_msgs/Float32MultiArray.h>
 
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

using namespace std;
 
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
ros::Publisher* pubptr = NULL;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
    ROS_INFO_STREAM("begin to exec!");
    ROS_INFO("Goal has %d points",goal->trajectory.points.size());
    std_msgs::Float32MultiArray msg;
    for(int i=0;i<goal->trajectory.points.size();i++)
        {
            msg.data.push_back(goal->trajectory.points[i].positions[0]);
            msg.data.push_back(goal->trajectory.points[i].positions[1]);
            msg.data.push_back(goal->trajectory.points[i].positions[2]);
            pubptr->publish(msg);
        }
    ROS_INFO("finished");
    as->setSucceeded();
    ROS_INFO_STREAM("exec end!");
}
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_server");
	ros::NodeHandle nh;
  
    ros::Publisher pub= nh.advertise<std_msgs::Float32MultiArray>("magician_robot/send_arm_data",100);
    pubptr = &pub;
	Server server(nh, "magician_arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
	server.start();
    ROS_INFO_STREAM("waiting for a goal!"<<"　"<<" ...");
	ros::spin();
}
